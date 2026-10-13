"""W144 정지 RGB·정렬 깊이의 배치 전후 비교. 로봇/ROS 명령을 보내지 않는다."""

import math

import cv2
import numpy as np


class PlacementMeasurement:
    """수평 눕힘 블록의 새 표면을 분리해 중심과 yaw를 탐색 측정한다."""

    def __init__(self, shape_tolerance_m, change_depth_m, search_radius_m,
                 plane_residual_m, max_tilt_rad):
        """탐색 문턱(m/rad)을 받는다. 유한한 양수만 허용, 외부 영향 없음.

        측정 정확도는 이 설정으로 보장하지 않는다. 현물 기준 검증 전 자동 재집기에는 쓰지 않는다.
        """
        values=(shape_tolerance_m,change_depth_m,search_radius_m,plane_residual_m,max_tilt_rad)
        if any(isinstance(v,bool) or not isinstance(v,(int,float)) or not math.isfinite(v) or v<=0 for v in values):
            raise ValueError('측정 설정은 유한한 양수')
        self.shape_tolerance_m=shape_tolerance_m
        self.change_depth_m=change_depth_m
        self.search_radius_m=search_radius_m
        self.plane_residual_m=plane_residual_m
        self.max_tilt_rad=max_tilt_rad

    @staticmethod
    def transform(record):
        """저장한 위치 m/xyzw를 강체 4x4로 변환한다. 불량 값은 ValueError, 외부 영향 없음."""
        q=np.asarray(record['quaternion_xyzw'],dtype=float)
        t=np.asarray(record['translation_m'],dtype=float)
        if q.shape!=(4,) or t.shape!=(3,) or not np.isfinite(np.r_[q,t]).all() or not np.isclose(q@q,1,atol=1e-3):
            raise ValueError('TF 위치/쿼터니언 오류')
        x,y,z,w=q/np.linalg.norm(q)
        result=np.eye(4)
        result[:3,:3]=[[1-2*(y*y+z*z),2*(x*y-z*w),2*(x*z+y*w)],
                      [2*(x*y+z*w),1-2*(x*x+z*z),2*(y*z-x*w)],
                      [2*(x*z-y*w),2*(y*z+x*w),1-2*(x*x+y*y)]]
        result[:3,3]=t
        return result

    @staticmethod
    def validate_transform(transform):
        """m 단위 강체 변환을 확인한다. 잘못된 회전/형식은 ValueError, 외부 영향 없음."""
        t=np.asarray(transform,dtype=float)
        if (t.shape!=(4,4) or not np.isfinite(t).all() or not np.allclose(t[3],[0,0,0,1])
                or not np.allclose(t[:3,:3].T@t[:3,:3],np.eye(3),atol=1e-6)
                or not np.isclose(np.linalg.det(t[:3,:3]),1,atol=1e-6)):
            raise ValueError('강체 변환 형식 오류')
        return t

    @staticmethod
    def wood_mask(rgb):
        """현재 목재 색 후보를 만든다. 색 문턱만으로 블록 존재/위치를 확정하지 않는다."""
        hsv=cv2.cvtColor(rgb,cv2.COLOR_BGR2HSV)
        return ((hsv[:,:,0]>8)&(hsv[:,:,0]<40)&(hsv[:,:,1]>35)&(hsv[:,:,2]>45))

    @staticmethod
    def rays(uv, info):
        """색 내참수/왜곡에서 광선을 만든다. 미지원 모델/내참수면 ValueError, 외부 영향 없음."""
        if info.get('distortion_model') not in ('','plumb_bob','rational_polynomial'):
            raise ValueError('미지원 왜곡 모델')
        k=np.asarray(info['k'],dtype=float).reshape(3,3)
        d=np.asarray(info['d'],dtype=float)
        if not np.isfinite(k).all() or not np.isfinite(d).all() or k[0,0]<=0 or k[1,1]<=0:
            raise ValueError('내참수 오류')
        xy=cv2.undistortPoints(np.asarray(uv,dtype=float).reshape(-1,1,2),k,d).reshape(-1,2)
        return np.column_stack([xy,np.ones(len(xy))])

    def frame(self, rgb, depth, info, transform):
        """RGB/정렬 깊이 m를 검증하고 base 점군 격자를 만든다. 잘못된 입력은 ValueError."""
        t=self.validate_transform(transform)
        if (rgb.dtype!=np.uint8 or rgb.ndim!=3 or rgb.shape[2]!=3
                or depth.shape!=rgb.shape[:2] or depth.ndim!=2
                or depth.shape!=(info['height'],info['width'])):
            raise ValueError('RGB/깊이/내참수 크기·형식 불일치')
        v,u=np.indices(depth.shape)
        camera=self.rays(np.column_stack([u.ravel(),v.ravel()]),info)*depth.ravel()[:,None]
        base=(camera@t[:3,:3].T+t[:3,3]).reshape(*depth.shape,3)
        valid=np.isfinite(depth)&(depth>0)
        base[~valid]=np.nan
        return dict(rgb=rgb,depth=depth,info=info,transform=t,base=base,
                    valid=valid,wood=self.wood_mask(rgb))

    def background(self, previous, current):
        """직전 관측을 현재 카메라로 3D 재투영한다. 빈 화소는 unknown, 가림은 z-buffer 처리."""
        pb=previous['base'][previous['valid']]
        wood=previous['wood'][previous['valid']]
        t=current['transform']
        pc=(pb-t[:3,3])@t[:3,:3]
        ok=np.isfinite(pc).all(axis=1)&(pc[:,2]>0)
        pc,wood=pc[ok],wood[ok]
        h,w=current['depth'].shape
        old_depth=np.full((h,w),np.nan)
        old_wood=np.zeros((h,w),bool)
        if len(pc):
            info=current['info']
            uv=cv2.projectPoints(pc,np.zeros(3),np.zeros(3),np.asarray(info['k']).reshape(3,3),
                                 np.asarray(info['d']))[0].reshape(-1,2)
            uv=np.rint(uv).astype(int)
            ok=(uv[:,0]>=0)&(uv[:,0]<w)&(uv[:,1]>=0)&(uv[:,1]<h)
            uv,pc,wood=uv[ok],pc[ok],wood[ok]
            linear=uv[:,1]*w+uv[:,0]
            order=np.lexsort((pc[:,2],linear))
            unique=np.r_[True,np.diff(linear[order])!=0] if len(order) else np.array([],bool)
            chosen=order[unique]
            old_depth.ravel()[linear[chosen]]=pc[chosen,2]
            old_wood.ravel()[linear[chosen]]=wood[chosen]
        return old_depth,old_wood

    def measure(self, current, size_m, search_center_m, previous=None):
        """현재 프레임과 검색 목표/크기 m → 탐색 중심/yaw/실패 이유. 외부 영향 없음.

        목표는 검색에만 사용한다. 중심/회전은 실제 경계에서 계산하며 후보가 둘이면 선택하지 않는다.
        수평 눕힘만 지원하고 기울기/부분가림/합쳐진 형상은 거절한다. 정확도 미검증 상태를 반환한다.
        """
        size=np.asarray(size_m,dtype=float);search=np.asarray(search_center_m,dtype=float)
        if size.shape!=(3,) or search.shape!=(3,) or not np.isfinite(np.r_[size,search]).all() or np.any(size<=0):
            raise ValueError('검색 위치/크기 오류')
        pb=current['base']
        roi=np.all(abs(pb-search)<self.search_radius_m,axis=2)
        mask=current['valid']&current['wood']&roi
        if previous is not None:
            old_depth,old_wood=self.background(previous,current)
            changed=np.isfinite(old_depth)&((~old_wood)|((old_depth-current['depth'])>self.change_depth_m))
            mask &= changed
        binary=cv2.morphologyEx(mask.astype(np.uint8)*255,cv2.MORPH_CLOSE,np.ones((3,3),np.uint8))
        contours,_=cv2.findContours(binary,cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)
        candidates=[];rejected=[]
        for contour in contours:
            if cv2.contourArea(contour)<30:continue
            inside=np.zeros(mask.shape,np.uint8)
            cv2.drawContours(inside,[contour],-1,255,-1)
            inside=cv2.erode(inside,np.ones((5,5),np.uint8))
            points=pb[(inside>0)&mask]
            if len(points)<30:
                rejected.append('SURFACE_POINTS');continue
            top=float(np.median(points[:,2]))
            residual=float(np.median(abs(points[:,2]-top)))
            central=points[abs(points[:,2]-top)<=max(self.plane_residual_m*3,1e-9)]
            if len(central)<30:
                rejected.append('SURFACE_POINTS');continue
            centered=central[:,:2]-np.mean(central[:,:2],axis=0)
            slopes=np.linalg.lstsq(np.column_stack([centered,np.ones(len(central))]),central[:,2],rcond=None)[0][:2]
            if residual>self.plane_residual_m or np.linalg.norm(slopes)>math.tan(self.max_tilt_rad):
                rejected.append('TILTED_OR_NOISY');continue
            ray=self.rays(contour[:,0,:],current['info'])@current['transform'][:3,:3].T
            camera=current['transform'][:3,3]
            if np.any(abs(ray[:,2])<1e-9):
                rejected.append('PLANE_RAY');continue
            distance=(top-camera[2])/ray[:,2]
            if np.any(distance<=0):
                rejected.append('PLANE_RAY');continue
            xy=(camera+ray*distance[:,None])[:,:2].astype(np.float32)
            rect=cv2.minAreaRect(xy)
            dims=np.sort(rect[1]);expected=np.sort(size[:2])
            if np.any(abs(dims-expected)>self.shape_tolerance_m):
                rejected.append('FOOTPRINT');continue
            area=cv2.contourArea(xy.reshape(-1,1,2))
            if area/(dims[0]*dims[1])<.9:
                rejected.append('INCOMPLETE_RECTANGLE');continue
            center=[float(rect[0][0]),float(rect[0][1]),top-size[2]/2]
            corners=cv2.boxPoints(rect).astype(float)
            edges=np.roll(corners,-1,axis=0)-corners
            long=edges[np.argmax(np.linalg.norm(edges,axis=1))]
            yaw=math.atan2(long[1],long[0])%math.pi
            candidates.append(dict(center_m=center,yaw_rad=yaw,footprint_m=dims.tolist(),
                surface_mad_m=residual,contour_pixels=contour[:,0,:].tolist()))
        if len(candidates)!=1:
            return dict(status='UNKNOWN',reason='AMBIGUOUS' if len(candidates)>1 else 'NO_COMPLETE_FOOTPRINT',
                        candidates=len(candidates),rejected=rejected,accuracy_validated=False)
        return dict(status='EXPLORATORY',accuracy_validated=False,
                    temporal=previous is not None,**candidates[0])

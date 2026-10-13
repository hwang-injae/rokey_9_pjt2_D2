"""받은 배치별 NPZ/TF를 오프라인 탐색 측정한다. 원자료/로봇/공유 문서를 수정하지 않는다."""

import argparse
import json
from pathlib import Path

import cv2
import numpy as np

from measurement import PlacementMeasurement


def main():
    """CLI 폴더/명시 보정 → JSON/대표 경계 그림. 검증 전 결과는 자동 재배치 입력으로 쓰지 않는다."""
    parser=argparse.ArgumentParser()
    parser.add_argument('source',type=Path)
    parser.add_argument('--out',type=Path,required=True)
    parser.add_argument('--size-m',nargs=3,type=float,required=True)
    parser.add_argument('--tcp-camera',type=Path)
    parser.add_argument('--calibration-units',choices=['m','mm'])
    parser.add_argument('--shape-tolerance-m',type=float,default=.005)
    parser.add_argument('--change-depth-m',type=float,default=.005)
    parser.add_argument('--search-radius-m',type=float,default=.10)
    parser.add_argument('--plane-residual-m',type=float,default=.002)
    parser.add_argument('--max-tilt-deg',type=float,default=3.)
    parser.add_argument('--max-skew-ms',type=float,default=50.)
    args=parser.parse_args()
    if args.tcp_camera and args.calibration_units is None:
        parser.error('보정 파일의 실제 이동 단위 --calibration-units 필요')
    if not np.isfinite(args.max_skew_ms) or args.max_skew_ms<=0:
        parser.error('max-skew-ms는 유한한 양수')
    if args.out.resolve()==args.source.resolve() or args.source.resolve() in args.out.resolve().parents:
        parser.error('원자료 바깥에 별도 분석 폴더를 지정하세요')
    args.out.mkdir(parents=True,exist_ok=True)
    detector=PlacementMeasurement(args.shape_tolerance_m,args.change_depth_m,args.search_radius_m,
                                  args.plane_residual_m,np.deg2rad(args.max_tilt_deg))
    ext=None
    if args.tcp_camera:
        ext=np.load(args.tcp_camera,allow_pickle=False).astype(float)
        if args.calibration_units=='mm':ext[:3,3]/=1000
        ext=detector.validate_transform(ext)
    targets=json.loads((args.source/'placements.json').read_text())
    rows=[];previous=None;previews=[]
    for target in targets:
        paths=sorted((args.source/f'{target["sequence"]:02d}_observe').glob('frame_*.json'))
        measured=[];failures=[];last=None
        for path in paths:
            try:
                meta=json.loads(path.read_text())
                skew=abs(meta['depth_stamp_s']-meta['color_stamp_s'])*1000
                if not np.isfinite(skew) or skew>args.max_skew_ms:raise ValueError('RGB/깊이 시각차 초과')
                rotation=detector.transform(dict(translation_m=[0,0,0],quaternion_xyzw=target['target_quaternion_xyzw']))
                if not np.allclose(rotation[:3,2],[0,0,1],atol=1e-3):raise ValueError('수평 눕힘 이외 자세는 미지원')
                tf=meta['base_camera'] if ext is None else meta['base_tcp']
                if not tf['ok']:raise ValueError('촬영 시각 TF 누락')
                transform=detector.transform(tf['value'])
                if ext is not None:transform=transform@ext
                with np.load(path.with_suffix('.npz'),allow_pickle=False) as data:
                    frame=detector.frame(data['color_bgr'].copy(),data['depth_m'].copy(),meta['camera_info'],transform)
                result=detector.measure(frame,args.size_m,target['target_center_m'],previous)
                last=frame
                if result['status']=='EXPLORATORY':
                    measured.append(dict(frame=path.stem,**result))
                    if path==paths[0]:
                        image=frame['rgb'].copy()
                        contour=np.array(result['contour_pixels'],np.int32).reshape(-1,1,2)
                        cv2.drawContours(image,[contour],-1,(0,0,255),1)
                        cv2.putText(image,str(target['sequence']),(20,30),cv2.FONT_HERSHEY_SIMPLEX,1,(0,0,255),2)
                        previews.append(image)
                else:failures.append(dict(frame=path.stem,**result))
            except (ValueError,KeyError,TypeError,OSError,cv2.error) as exc:
                failures.append(dict(frame=path.stem,reason=str(exc)))
        row=dict(sequence=target['sequence'],part_id=target['block_id'],total=len(paths),
                 measured=len(measured),status='EXPLORATORY' if measured else 'UNKNOWN',
                 correction_allowed=False,measurements=measured,failures=failures)
        if measured:
            centers=np.array([m['center_m'] for m in measured])
            row.update(center_m=np.median(centers,axis=0).tolist(),
                       delta_mm=((np.median(centers,axis=0)-target['target_center_m'])*1000).tolist(),
                       repeat_range_mm=(np.ptp(centers,axis=0)*1000).tolist())
        rows.append(row)
        previous=last  # 관측 누락 때 이전 배치의 배경을 정상 자료로 대신 사용하지 않는다.
    output=dict(scope='수평 눕힘 배치 전후 RGB/깊이 탐색. 실측 정확도 미검증, 자동 재집기 금지',
                source=str(args.source),calibration=str(args.tcp_camera) if ext is not None else 'captured TF',
                parameters=vars(args)|{'source':str(args.source),'out':str(args.out),'tcp_camera':str(args.tcp_camera)},rows=rows)
    (args.out/'measurement.json').write_text(json.dumps(output,ensure_ascii=False,indent=2,allow_nan=False))
    if previews and not cv2.imwrite(str(args.out/'preview.jpg'),np.hstack(previews)):
        raise OSError('대표 영상 저장 실패')
    print(json.dumps([{k:r[k] for k in ('sequence','total','measured','status')}|
                      {'delta_mm':r.get('delta_mm')} for r in rows],ensure_ascii=False))


if __name__=='__main__':
    main()

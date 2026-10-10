# -*- coding: utf-8 -*-
"""/api/designs — 설계 목록(트리) · 설계 하나(3D 보기) · 조립 기록 · 설계 규칙 숫자 (web/README 2장, SDD 6.8, W111).

읽기만 한다 — 저장은 DesignStore 한 곳. 생성 · 고르기 · 스캔 저장(W108 · W112 · W116)은 뒤에 붙는다.
트리는 화면이 목록의 parent_id 로 만든다(따로 /tree 를 두지 않는다 — 같은 내용을 두 번 내보내지 않게).
"""
from fastapi import APIRouter, HTTPException, Request

router = APIRouter(prefix='/api/designs')


@router.get('')
def list_designs(request: Request, family: str | None = None):
    """설계 요약 목록 [{design_id, family, version, parent_id, made_by, block_count, size_mm, last_build}]. family 로 거를 수 있다."""
    return request.app.state.store.list_designs(family)


@router.get('/rules')
def rules(request: Request):
    """화면이 쓰는 설계 규칙 숫자(robot.yaml 에서 backend 가 읽은 값 — 10/9 PL E-78). 3D 블록 크기 · 작업공간(mm)."""
    r = request.app.state.rules
    return {'block_size_mm': [round(v * 1000, 3) for v in r['block_size_m']],
            'assembly_area_half_mm': round(r['assembly_area_half_m'] * 1000, 3)}


@router.get('/{design_id}')
def get_design(design_id: str, request: Request):
    """design/2.0 하나. 없으면 404, 저장된 형식이 다르면 409(옛 형식은 변환하지 않는다 — E-69)."""
    try:
        return request.app.state.store.get_design(design_id)
    except KeyError:
        raise HTTPException(404, f'{design_id} 설계가 없다') from None
    except ValueError as e:
        raise HTTPException(409, str(e)) from None


@router.get('/{design_id}/builds')
def builds(design_id: str, request: Request):
    """이 설계의 조립 기록(build/1 전부, 최근 것 먼저). 설계가 없으면 404 — 기록이 없으면 빈 목록."""
    store = request.app.state.store
    try:
        store.get_design(design_id)
    except KeyError:
        raise HTTPException(404, f'{design_id} 설계가 없다') from None
    except ValueError as e:
        raise HTTPException(409, str(e)) from None
    return store.builds_for(design_id)

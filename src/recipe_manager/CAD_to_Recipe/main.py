"""CAD -> assembly recipe.
inspect: CAD -> model JSON + plan template.  build: CAD + filled plan -> recipe JSON + placement table."""
import argparse
from pathlib import Path
import sys

from RecipeManager import RecipeManager

# 모든 출력은 recipe_manager/recipes/ 폴더에 저장한다. 어느 폴더에서 실행해도 같은 곳이다.
# 파일 이름은 모델 ID로 정한다: <모델ID>.model.json, .plan.json, .recipe.json, .placements.csv
OUTPUT_DIR = Path(__file__).resolve().parent.parent / 'recipes'

USAGE_EXAMPLES = """
실행 인자가 없습니다. 저장소 루트의 터미널에서 아래와 같이 실행하세요.

1) CAD에서 모델과 계획 양식 생성:
   python3 recipe_manager/CAD_to_Recipe/main.py inspect recipe_manager/cad/lv1_bench.dxf --model-id LV1
2) recipe_manager/recipes/LV1.plan.json의 sequence, stage, grasp를 확인·입력
3) 레시피 생성:
   python3 recipe_manager/CAD_to_Recipe/main.py build recipe_manager/cad/lv1_bench.dxf recipe_manager/recipes/LV1.plan.json
"""


def main(argv=None):
    """명령줄 입구(실행 진입점): 인자를 해석해 RecipeManager의 inspect_cad 또는 build_recipe를 부른다.

    입력:
        argv: 명령 인자 목록. None이면 sys.argv[1:]
    출력: 종료 코드 (오류는 parser.exit로 2를 낸다)
    바깥 영향: 출력 폴더에 파일을 쓴다(RecipeManager), 같은 이름 파일이 있으면 터미널에서 묻는다.
    실패: ValueError·KeyError·TypeError·OSError·ImportError는 메시지와 함께 종료 코드 2로 끝낸다.
    """
    # 명령 정의
    parser = argparse.ArgumentParser(description=__doc__)
    sub = parser.add_subparsers(dest='command', required=True)
    inspect = sub.add_parser('inspect', help='CAD -> recipes/<model-id>.model.json + recipes/<model-id>.plan.json')
    inspect.add_argument('cad')
    inspect.add_argument('--model-id', required=True)
    inspect.add_argument('--revision', default='1')
    build = sub.add_parser('build', help='CAD + filled plan -> recipes/<model-id>.recipe.json + .placements.csv')
    build.add_argument('cad')
    build.add_argument('plan')


    # VS Code 실행 버튼처럼 인자 없이 실행하면 사용법만 보여 주고 정상 종료한다.
    argv = sys.argv[1:] if argv is None else argv
    if not argv:
        parser.print_help()
        print(USAGE_EXAMPLES)
        return 0
    args = parser.parse_args(argv)


    # 작업 실행. 오류는 종료 코드 2로 바꾼다.
    manager = RecipeManager(OUTPUT_DIR)
    try:
        if args.command == 'inspect':
            manager.inspect_cad(args.cad, args.model_id, args.revision)
        else:
            manager.build_recipe(args.cad, args.plan)
    except ImportError as exc:
        parser.exit(2, f'{exc}\nCAD 라이브러리가 필요합니다: pip install -r recipe_manager/requirements.txt\n')
    except (ValueError, KeyError, TypeError, OSError) as exc:
        parser.exit(2, f'{exc}\n')
    return 0


if __name__ == '__main__':
    sys.exit(main())

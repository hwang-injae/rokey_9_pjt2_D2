"""CAD → 조립 레시피.
inspect: CAD → 블록 표(화면) + 계획 양식(CAD 옆 <모델ID>_plan.json).  build: CAD + 계획 → recipes/ 에 레시피 JSON + 배치표 CSV."""
import argparse
import csv
import hashlib
import io
import json
import sys
from pathlib import Path

from cad_reader import CadReader
from recipe_builder import RecipeBuilder

# 레시피 · 배치표는 늘 src/recipe_manager/recipes/ 에 저장한다(어느 폴더에서 실행해도 같다).
# d2_bringup 이 이 폴더의 *_recipe.json 을 설치하고 task · 비전 · run_recipe 가 읽는다.
RECIPE_DIR = Path(__file__).resolve().parents[1] / 'recipes'

USAGE_EXAMPLES = """
실행 인자가 없습니다. 저장소 루트의 터미널에서 아래와 같이 실행하세요.

1) CAD에서 계획 양식 만들기 (CAD 파일 옆에 저장):
   python3 src/recipe_manager/recipe_manager/main.py inspect src/recipe_manager/cads/001_chair_bench.dxf --model-id 001_CHAIR_BENCH
2) src/recipe_manager/cads/001_CHAIR_BENCH_plan.json 의 sequence, stage, grasp 확인 · 입력
3) 레시피 만들기:
   python3 src/recipe_manager/recipe_manager/main.py build src/recipe_manager/cads/001_chair_bench.dxf src/recipe_manager/cads/001_CHAIR_BENCH_plan.json
"""


class RecipeManager:
    """명령(inspect · build)을 받아 CAD를 읽고(CadReader) 레시피를 만들어(RecipeBuilder) 파일로 저장한다.

    나중에 HMI 화면이 부르더라도 이 클래스를 쓰면 된다. 파일 저장 · 덮어쓰기 질문은 여기에만 있다.
    """

    def __init__(self, recipe_dir):
        """레시피 · 배치표를 저장할 폴더를 정한다.

        입력:
            recipe_dir: 저장 폴더 (보통 RECIPE_DIR)
        """
        self.recipe_dir = Path(recipe_dir)
        self.reader = CadReader()
        self.builder = RecipeBuilder()

    def inspect_cad(self, cad_path, model_id, revision='1'):
        """CAD → 블록 표를 화면에 보여 주고, 계획 양식을 CAD 파일 옆 <모델ID>_plan.json 으로 저장한다.

        계획의 sequence · stage · grasp(파지 방법)를 사람이 확인 · 입력한 뒤 build 를 실행한다.
        DXF 속성(SEQ · STAGE · GRASP)이 있으면 미리 채워진다.

        입력:
            cad_path: CAD 파일 경로
            model_id: 모델 ID (예: 001_CHAIR_BENCH — 계획 · 레시피 파일 이름과 block_id 앞부분이 된다)
            revision: 모델 버전
        바깥 영향: 계획 파일을 쓰고(save_file), 블록 표를 터미널에 출력한다.
        실패: CAD를 읽거나 모델을 만들 수 없으면 ValueError, CAD 라이브러리가 없으면 ImportError.
        """
        # CAD → 모델 + 계획 양식
        model, hints = self.load_model(cad_path, model_id, revision)
        plan = self.builder.make_plan_template(model, hints)


        # 블록 표: 사람이 계획의 instance_id 를 CAD 위치와 대조한다(옛 model.json 대신)
        print('블록 ID              중심 x, y, z (mm)       치수 LxWxT   SEQ STAGE GRASP')
        sizes = {part['part_id']: part['size_mm'] for part in model['parts']}
        for instance in model['instances']:
            hint = hints.get(instance['instance_id'], {})
            center = '({:g}, {:g}, {:g})'.format(*instance['center_mm'])
            size = 'x'.join(f'{v:g}' for v in sizes[instance['part_id']])
            print(f"{instance['instance_id']:<21}{center:<24}{size:<13}{hint.get('SEQ', '-'):<4}"
                  f"{hint.get('STAGE', '-'):<6}{hint.get('GRASP', '-')}")
        print()


        # 계획 양식 저장 + 사람이 채울 단계 수 안내
        self.save_file(Path(cad_path).parent / f'{model_id}_plan.json', self.make_json_text(plan))
        missing = sum(None in (step['sequence'], step['stage'], step['grasp']) for step in plan['steps'])
        print(f'부품 {len(model["instances"])}개, 미입력 단계 {missing}개. 계획 파일의 sequence, stage, grasp를 확인하세요.')

    def build_recipe(self, cad_path, plan_path):
        """CAD + 계획 → 검증 → recipes/<모델ID>_recipe.json + <모델ID>_placements.csv

        CAD를 다시 읽어 모델을 만든다. 계획의 CAD 해시와 다르면 거부한다(계획을 쓴 뒤 CAD가 바뀜).

        입력:
            cad_path: CAD 파일 경로
            plan_path: 사람이 작성한 계획 파일 경로
        바깥 영향: recipe_dir 에 두 파일을 쓰고(save_file), 배치표를 터미널에 출력한다.
        실패: 계획 형식 · 내용이 틀리거나 CAD 해시가 다르면 ValueError (파일을 쓰지 않는다).
        """
        # 계획 읽기 → CAD 다시 읽기 → 검증해 레시피 생성
        plan = json.loads(Path(plan_path).read_text(encoding='utf-8'))
        model, _ = self.load_model(cad_path, plan.get('model_id'), plan.get('model_revision'))
        recipe = self.builder.build_recipe(model, plan)
        rows = self.builder.make_placement_rows(recipe)
        print(f'검증 완료: {recipe["recipe_id"]}, {len(recipe["steps"])}개 배치\n')


        # 배치표 터미널 출력. 한글은 화면에서 두 칸을 차지해 머리글 공백은 직접 맞췄다.
        print('순번 단계  블록                  치수 LxWxT  중심 x, y, z (mm)       방향 L/W/T  파지 방법    닫힘 축    지지 부품')
        for row in rows:
            size = f"{row['size_L_mm']:g}x{row['size_W_mm']:g}x{row['size_T_mm']:g}"
            center = f"({row['center_x_mm']:g}, {row['center_y_mm']:g}, {row['center_z_mm']:g})"
            axes = f"{row['axis_L']}/{row['axis_W']}/{row['axis_T']}"
            supports = ', '.join(row['support_block_ids']) or '책상'
            print(f"{row['sequence']:>4} {row['stage']:>4}  {row['block_id']:<22}{size:<12}"
                  f"{center:<24}{axes:<12}{row['grasp']:<13}{row['grasp_axis']:<11}{supports}")
        print()


        # 배치표 CSV. 열 이름은 make_placement_rows 키 그대로, 받침이 여럿이면 ';'로 잇는다.
        buffer = io.StringIO()
        writer = csv.DictWriter(buffer, fieldnames=list(rows[0]), lineterminator='\n')
        writer.writeheader()
        for row in rows:
            writer.writerow({**row, 'support_instance_ids': ';'.join(row['support_instance_ids']),
                             'support_block_ids': ';'.join(row['support_block_ids'])})


        # 레시피와 배치표 저장
        model_id = model['model_id']
        self.save_file(self.recipe_dir / f'{model_id}_recipe.json', self.make_json_text(recipe))
        self.save_file(self.recipe_dir / f'{model_id}_placements.csv', buffer.getvalue())

    def load_model(self, cad_path, model_id, revision):
        """CAD를 읽어 모델과 DXF 속성 힌트를 만든다.

        입력:
            cad_path: CAD 파일 경로
            model_id, revision: 모델 ID와 버전
        출력: (모델 dict, {instance_id: {'SEQ': ..., 'STAGE': ..., 'GRASP': ...}})
        실패: CadReader · RecipeBuilder.make_model 의 ValueError · ImportError 그대로.
        """
        boxes = self.reader.read_cad(cad_path)
        sha256 = hashlib.sha256(Path(cad_path).read_bytes()).hexdigest()
        model = self.builder.make_model(boxes, model_id, revision, Path(cad_path).name, sha256)
        return model, {box['id']: box['hints'] for box in boxes if box['hints']}

    def make_json_text(self, data):
        """JSON 파일에 쓸 문자열. 한글은 그대로, 들여쓰기 2칸, 끝에 줄바꿈.

        입력:
            data: 저장할 dict
        출력: 문자열
        """
        return json.dumps(data, ensure_ascii=False, indent=2) + '\n'

    def save_file(self, path, text):
        """text를 파일로 저장한다. 같은 이름이 있으면 덮어쓰기 / 다른 이름으로 저장 / 취소 중에서 고르게 한다.

        입력:
            path: 저장할 파일 경로
            text: 파일 내용
        출력: 실제로 저장한 경로. 취소하면 None
        바깥 영향: 파일을 쓰고, 같은 이름이 있으면 input()으로 묻는다.
        실패: 입력을 받을 수 없으면(EOFError) 기존 파일을 지키려고 저장하지 않고 None.
        """
        # 같은 이름 파일이 있으면 묻는다. 다른 이름을 골랐는데 그 파일도 있으면 다시 묻는다.
        path = Path(path)
        while path.exists():
            try:
                choice = input(f'{path} 파일이 이미 있습니다. [1] 덮어쓰기  [2] 다른 이름으로 저장  [3] 취소 > ').strip()
                if choice == '1':
                    break
                if choice == '2':
                    new_name = input('새 파일 이름 (같은 폴더에 저장) > ').strip()
                    if new_name:
                        path = path.parent / new_name
                elif choice == '3':
                    print(f'저장 취소: {path}')
                    return None
            except EOFError:
                # 자동 실행처럼 입력을 받을 수 없으면 기존 파일을 지키려고 취소한다.
                print(f'저장 취소 (입력 없음): {path}')
                return None


        # 저장
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_text(text, encoding='utf-8')
        print(f'저장: {path}')
        return path


def main(argv=None):
    """명령줄 입구(실행 진입점): 인자를 해석해 RecipeManager의 inspect_cad 또는 build_recipe를 부른다.

    입력:
        argv: 명령 인자 목록. None이면 sys.argv[1:]
    출력: 종료 코드 (오류는 parser.exit로 2)
    바깥 영향: 파일을 쓴다(RecipeManager). 같은 이름 파일이 있으면 터미널에서 묻는다.
    실패: ValueError · KeyError · TypeError · OSError · ImportError는 메시지와 함께 종료 코드 2.
    """
    # 명령 정의
    parser = argparse.ArgumentParser(description=__doc__)
    sub = parser.add_subparsers(dest='command', required=True)
    inspect = sub.add_parser('inspect', help='CAD -> 블록 표 + CAD 옆 <model-id>_plan.json')
    inspect.add_argument('cad')
    inspect.add_argument('--model-id', required=True)
    inspect.add_argument('--revision', default='1')
    build = sub.add_parser('build', help='CAD + 계획 -> recipes/<model-id>_recipe.json + _placements.csv')
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
    manager = RecipeManager(RECIPE_DIR)
    try:
        if args.command == 'inspect':
            manager.inspect_cad(args.cad, args.model_id, args.revision)
        else:
            manager.build_recipe(args.cad, args.plan)
    except ImportError as exc:
        parser.exit(2, f'{exc}\nCAD 라이브러리가 필요합니다: pip install -r src/recipe_manager/requirements.txt\n')
    except (ValueError, KeyError, TypeError, OSError) as exc:
        parser.exit(2, f'{exc}\n')
    return 0


if __name__ == '__main__':
    sys.exit(main())

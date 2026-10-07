"""CAD(DXF) → 레시피 (E-52).
build: cads/<모델ID 소문자>.dxf → recipes/<모델ID>_structure.json + _recipe.json + _placements.csv."""
import argparse
import csv
import hashlib
import io
import sys
from pathlib import Path

from cad_reader import CadReader
from recipe_builder import RecipeBuilder

# 레시피 · 배치표는 늘 src/recipe_manager/recipes/ 에 저장한다(어느 폴더에서 실행해도 같다).
# d2_bringup 이 이 폴더의 *_structure.json · *_recipe.json 을 설치하고 task · 비전 · run_recipe 가 읽는다.
RECIPE_DIR = Path(__file__).resolve().parents[1] / 'recipes'

USAGE_EXAMPLES = """
실행 인자가 없습니다. 저장소 루트의 터미널에서 아래와 같이 실행하세요.

   python3 src/recipe_manager/recipe_manager/main.py build src/recipe_manager/cads/001_chair_bench.dxf

순서 · 단계 · 잡기는 DXF INSERT 속성(SEQ · STAGE · GRASP), 블록 이름은 INSERT 블록 이름(예 LEG_001_01)에서 읽는다.
바꾸려면 CAD를 고친 뒤 다시 build 한다(계획 파일은 없다 — E-52).
"""


class RecipeManager:
    """명령(build)을 받아 DXF를 읽고(CadReader) 구조 · 조립 방법을 만들어(RecipeBuilder) 파일로 저장한다.

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

    def build_recipe(self, cad_path):
        """DXF → 검증 → recipes/<모델ID>_structure.json + _recipe.json + _placements.csv

        모델 ID = CAD 파일 이름을 대문자로(작명 규칙: CAD 파일 이름 = 모델 ID 소문자, 예 001_chair_bench.dxf → 001_CHAIR_BENCH).

        입력:
            cad_path: DXF 파일 경로
        바깥 영향: recipe_dir 에 세 파일을 쓰고(save_file), 배치표를 터미널에 출력한다.
        실패: CAD · 이름 · 속성 · 배치가 틀리면 ValueError (파일을 쓰지 않는다), ezdxf가 없으면 ImportError.
        """
        # DXF → 구조 → 조립 방법. structure_sha256 은 저장할 구조 글자 그대로의 바이트로 계산한다.
        cad_path = Path(cad_path)
        model_id = cad_path.stem.upper()
        structure, hints = self.load_structure(cad_path, model_id)
        structure_text = self.builder.make_json_text(structure)
        recipe = self.builder.make_recipe(structure, hashlib.sha256(structure_text.encode('utf-8')).hexdigest(), hints)
        rows = self.builder.make_placement_rows(structure, recipe)
        print(f'검증 완료: {model_id}, {len(recipe["steps"])}개 배치\n')


        # 배치표 터미널 출력. 한글은 화면에서 두 칸을 차지해 머리글 공백은 직접 맞췄다.
        print('순번 단계  블록           치수 LxWxT  중심 x, y, z (mm)       방향 L/W/T  파지 방법    닫힘 축    받침')
        for row in rows:
            size = f"{row['size_L_mm']:g}x{row['size_W_mm']:g}x{row['size_T_mm']:g}"
            center = f"({row['center_x_mm']:g}, {row['center_y_mm']:g}, {row['center_z_mm']:g})"
            axes = f"{row['axis_L']}/{row['axis_W']}/{row['axis_T']}"
            supports = ', '.join(row['supports']) or '책상'
            print(f"{row['sequence']:>4} {row['stage']:>4}  {row['block']:<15}{size:<12}"
                  f"{center:<24}{axes:<12}{row['grasp']:<13}{row['grasp_axis']:<11}{supports}")
        print()


        # 배치표 CSV. 열 이름은 make_placement_rows 키 그대로, 받침이 여럿이면 ';'로 잇는다.
        buffer = io.StringIO()
        writer = csv.DictWriter(buffer, fieldnames=list(rows[0]), lineterminator='\n')
        writer.writeheader()
        for row in rows:
            writer.writerow({**row, 'supports': ';'.join(row['supports'])})


        # 구조 · 조립 방법 · 배치표 저장
        self.save_file(self.recipe_dir / f'{model_id}_structure.json', structure_text)
        self.save_file(self.recipe_dir / f'{model_id}_recipe.json', self.builder.make_json_text(recipe))
        self.save_file(self.recipe_dir / f'{model_id}_placements.csv', buffer.getvalue())

    def load_structure(self, cad_path, model_id):
        """DXF를 읽어 구조와 블록마다의 CAD 속성(순서 · 단계 · 잡기)을 만든다.

        입력:
            cad_path: DXF 파일 경로
            model_id: 모델 ID
        출력: (구조 dict, {블록 이름: {'SEQ': ..., 'STAGE': ..., 'GRASP': ...}})
        실패: CadReader · RecipeBuilder.make_structure 의 ValueError · ImportError 그대로.
        """
        boxes = self.reader.read_dxf(cad_path)
        sha256 = hashlib.sha256(Path(cad_path).read_bytes()).hexdigest()
        structure = self.builder.make_structure(boxes, model_id, Path(cad_path).name, sha256)
        return structure, {box['block']: box['hints'] for box in boxes}

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
    """명령줄 입구(실행 진입점): 인자를 해석해 RecipeManager.build_recipe 를 부른다.

    입력:
        argv: 명령 인자 목록. None이면 sys.argv[1:]
    출력: 종료 코드 (오류는 parser.exit로 2)
    바깥 영향: 파일을 쓴다(RecipeManager). 같은 이름 파일이 있으면 터미널에서 묻는다.
    실패: ValueError · KeyError · TypeError · OSError · ImportError는 메시지와 함께 종료 코드 2.
    """
    # 명령 정의
    parser = argparse.ArgumentParser(description=__doc__)
    sub = parser.add_subparsers(dest='command', required=True)
    build = sub.add_parser('build', help='DXF -> recipes/<model-id>_structure.json + _recipe.json + _placements.csv')
    build.add_argument('cad')


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
        manager.build_recipe(args.cad)
    except ImportError as exc:
        parser.exit(2, f'{exc}\nCAD 라이브러리가 필요합니다: pip install -r src/recipe_manager/requirements.txt\n')
    except (ValueError, KeyError, TypeError, OSError) as exc:
        parser.exit(2, f'{exc}\n')
    return 0


if __name__ == '__main__':
    sys.exit(main())

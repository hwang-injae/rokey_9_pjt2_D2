import csv
import hashlib
import io
import json
from pathlib import Path

from cad.CadReader import CadReader
from recipe_manager.CAD_to_Recipe.recipe.model.defined.Model import Model
from recipe.defined.Plan import Plan
from recipe.defined.Recipe import Recipe


class RecipeManager:
    """레시피를 만들고 저장하는 작업을 관리한다.

    지금 입력은 CAD 하나다: CAD → 모델·계획 양식(inspect_cad) → 사람이 계획 작성 → 레시피(build_recipe).
    다른 입력(예: 다른 설계 데이터)이 생기면 그 입력을 읽는 메서드를 여기에 더한다.
    명령줄(main.py)이 지금 이 클래스를 쓰고, 나중에 HMI 화면도 같은 클래스를 부르면 된다.
    모든 결과 파일은 output_dir 폴더에 모델 ID 이름으로 저장한다.
    """

    def __init__(self, output_dir):
        """결과 파일을 저장할 폴더를 정한다.

        입력:
            output_dir: 결과 파일을 저장할 폴더
        """
        self.output_dir = Path(output_dir)

    def inspect_cad(self, cad_path, model_id, revision='1'):
        """CAD → <모델ID>.model.json + <모델ID>.plan.json

        모델 파일은 사람이 인스턴스 ID와 위치를 대조하는 참고용이다.
        계획 파일의 sequence·stage·grasp(파지 방법)를 사람이 확인·입력한 뒤 build를 실행한다.

        입력:
            cad_path: CAD 파일 경로
            model_id: 모델 이름 (출력 파일 이름이 된다)
            revision: 모델 버전
        바깥 영향: output_dir에 두 파일을 쓰고(save_file), 결과를 터미널에 출력한다.
        실패: CAD를 읽거나 모델을 만들 수 없으면 ValueError, 라이브러리가 없으면 ImportError.
        """
        # CAD → 모델 + 계획 양식(DXF 속성 힌트로 미리 채움)
        model, hints = self.load_model(cad_path, model_id, revision)
        plan = Plan.create_template(model, hints)


        # 두 파일 저장
        self.save_file(self.output_dir / f'{model_id}.model.json', self.make_json_text(model.convert_to_dict()))
        self.save_file(self.output_dir / f'{model_id}.plan.json', self.make_json_text(plan.convert_to_dict()))


        # 사람이 채울 단계 수 안내
        missing = sum(None in (step.sequence, step.stage, step.grasp) for step in plan.steps)
        print(f'부품 {len(model.instances)}개, 미입력 단계 {missing}개. '
              '계획 파일의 sequence, stage, grasp를 확인하세요.')

    def build_recipe(self, cad_path, plan_path):
        """CAD + 작성한 계획 → 검증 → <모델ID>.recipe.json + <모델ID>.placements.csv

        CAD를 다시 읽어 모델을 만든다. 계획의 CAD 해시와 다르면 Recipe.build_from_plan가 거부한다.

        입력:
            cad_path: CAD 파일 경로
            plan_path: 사람이 작성한 plan.json 경로
        바깥 영향: output_dir에 두 파일을 쓰고(save_file), 배치표를 터미널에 출력한다.
        실패: 계획 형식·내용이 틀리거나 CAD 해시가 다르면 ValueError(파일을 쓰지 않는다).
        """
        # 계획 읽기 → CAD 다시 읽기 → 검증해 레시피 생성
        plan = Plan.create_from_dict(json.loads(Path(plan_path).read_text(encoding='utf-8')))
        model, _ = self.load_model(cad_path, plan.model_id, plan.model_revision)
        recipe = Recipe.build_from_plan(model, plan)
        print(f'검증 완료: {recipe.recipe_id}, {len(recipe.steps)}개 배치\n')
        rows = recipe.make_placement_rows()


        # 배치표 터미널 출력. 한글은 화면에서 두 칸을 차지하므로 머리글의 공백은 직접 맞췄다.
        print('순번 단계  부품            치수 LxWxT  중심 x, y, z (mm)       방향 L/W/T  파지 방법    닫힘 축    지지 부품')
        for row in rows:
            size = f"{row['size_L_mm']:g}x{row['size_W_mm']:g}x{row['size_T_mm']:g}"
            center = f"({row['center_x_mm']:g}, {row['center_y_mm']:g}, {row['center_z_mm']:g})"
            axes = f"{row['axis_L']}/{row['axis_W']}/{row['axis_T']}"
            supports = ', '.join(row['support_instance_ids']) or '책상'
            print(f"{row['sequence']:>4} {row['stage']:>4}  {row['instance_id']:<16}{size:<12}"
                  f"{center:<24}{axes:<12}{row['grasp']:<13}{row['grasp_axis']:<11}{supports}")
        print()


        # 배치표 CSV 문자열. 열 이름은 make_placement_rows의 키 그대로, 지지 부품이 여러 개면 ';'로 잇는다.
        buffer = io.StringIO()
        writer = csv.DictWriter(buffer, fieldnames=list(rows[0]), lineterminator='\n')
        writer.writeheader()
        for row in rows:
            writer.writerow({**row, 'support_instance_ids': ';'.join(row['support_instance_ids'])})


        # 레시피와 배치표 저장
        self.save_file(self.output_dir / f'{model.model_id}.recipe.json', self.make_json_text(recipe.convert_to_dict()))
        self.save_file(self.output_dir / f'{model.model_id}.placements.csv', buffer.getvalue())

    @staticmethod
    def load_model(cad_path, model_id, revision):
        """CAD를 읽어 Model과 DXF 속성 힌트를 만든다.

        입력:
            cad_path: CAD 파일 경로
            model_id, revision: 모델 이름과 버전
        출력: (Model, {instance_id: {'SEQ': ..., 'STAGE': ..., 'GRASP': ...}})
        """
        # CAD 읽기와 파일 해시
        raw_boxes = CadReader.read_cad(cad_path)
        sha256 = hashlib.sha256(Path(cad_path).read_bytes()).hexdigest()


        # 모델 생성과 힌트 모으기
        model = Model.create_from_raw_boxes(raw_boxes, model_id, revision, Path(cad_path).name, sha256)
        hints = {raw.source_id: raw.hints for raw in raw_boxes if raw.hints}
        return model, hints

    @staticmethod
    def make_json_text(data):
        """JSON 파일에 쓸 문자열을 만든다. 한글은 그대로, 들여쓰기 2칸.

        입력:
            data: 저장할 dict
        출력: 문자열 (끝에 줄바꿈)
        """
        return json.dumps(data, ensure_ascii=False, indent=2) + '\n'

    def save_file(self, path, text):
        """text를 파일로 저장한다.

        같은 이름의 파일이 이미 있으면 덮어쓰기 / 다른 이름으로 저장 / 취소 중에서 고르게 한다.
        나중에 HMI 화면에서는 이 질문 부분을 저장 확인 창으로 바꾸면 된다.

        입력:
            path: 저장할 파일 경로
            text: 파일 내용
        출력: 실제로 저장한 경로. 취소하면 None
        바깥 영향: 파일을 쓰고, 같은 이름이 있으면 input()으로 묻는다.
        실패: 입력을 받을 수 없으면(EOFError) 기존 파일을 지키려고 저장하지 않고 None.
        """
        # 같은 이름 파일이 있으면 사용자에게 묻는다. 다른 이름을 골랐는데 그 파일도 있으면 다시 묻는다.
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
                # 그 밖의 입력은 무시하고 다시 묻는다.
            except EOFError:
                # 입력을 받을 수 없는 환경(자동 실행 등)에서는 기존 파일을 지키기 위해 취소한다.
                print(f'저장 취소 (입력 없음): {path}')
                return None


        # 저장
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_text(text, encoding='utf-8')
        print(f'저장: {path}')
        return path

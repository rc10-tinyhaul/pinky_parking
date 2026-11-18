# 기여 안내

## 작업 흐름

1. `dev`에서 개인 브랜치를 만듭니다. 이름은 `이름/기능`입니다(예: `minsu/docking-aruco`).
2. 맡은 패키지 안에서 작업합니다. 다른 패키지를 고쳐야 하면 PR 설명에 이유를 적습니다.
3. 확인합니다(작업공간 루트에서).

   ```bash
   colcon build --symlink-install --packages-up-to $(colcon list --base-paths src/pinky_parking --names-only)
   colcon test --packages-select $(colcon list --base-paths src/pinky_parking --names-only) && colcon test-result
   src/pinky_parking/scripts/smoke_test.sh
   ```

4. `dev`로 PR을 엽니다. `main`은 `dev`가 안정됐을 때만 병합합니다.

## 커밋

- 한 커밋에 한 가지 목적만 담습니다.
- 제목은 [Conventional Commits](https://www.conventionalcommits.org/ko/) 형식에 한글 설명입니다: `feat(docking): 마커 기준 정렬 추가`
- 본문에는 왜 그렇게 했는지를 적습니다.

## 코드

- ROS 2 스타일을 따릅니다(`ament_flake8`, `ament_pep257`. `colcon test`가 검사합니다).
- 좌표, 이름, 도메인 같은 값은 코드가 아니라 `config/` YAML에 둡니다.
- 새 패키지는 `pinky_parking_` 접두어로 저장소 루트에 만듭니다.

  ```bash
  ros2 pkg create --build-type ament_python --license Apache-2.0 pinky_parking_<역할>
  ```

## 문서

한국어 "~합니다"체로 간결하게 씁니다. 패키지 README에는 launch 인자, 실행 파일, 설정 형식을 둡니다.

## 안전

실물 로봇을 움직이는 명령은 사람이 확인한 뒤 실행합니다. 실물 첫 시험은 낮은 속도 제한으로 합니다.

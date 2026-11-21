
실행 순서 요약

시나리오는 [1]~[8] 순서로 한 번에 이어지지만, 이미 띄워 둔 가제보에서는 구간을 나눠 이어서 실행합니다. 각 단계는 앞 단계가 끝난 현재 위치에서 시작합니다.

순서	명령	하는 일	시작 전에 로봇이 있어야 할 위치
0	터미널 1: ./scripts/gazebo_sim.sh remove → ./scripts/gazebo_sim.sh --all	월드와 로봇 a, b를 칸 기본 위치에 새로 띄움	주차 칸(자동)
1	./scripts/demo_scenario.sh --upto 1 --attach	[1] a, b 동시 출차 (a는 1번 트랙, b는 2번 트랙)	주차 칸
2	./scripts/demo_scenario.sh --from 2 --upto 4 --attach	[2] 일반주행 1바퀴, [3] a 1→2 차선변경, [4] 2번 트랙 간격 조절	a는 1번, b는 2번 트랙
3	./scripts/demo_scenario.sh --from 5 --upto 6 --attach	[5] b 2→1 차선변경, [6] b 입차 (P_DN)	a, b 모두 2번 트랙
4	./scripts/demo_scenario.sh --from 7 --upto 8 --attach	[7] a 2→1 차선변경, [8] a 입차 (P_UP)	a는 2번 트랙 (b는 입차 완료)

터미널 1에서 ./scripts/gazebo_sim.sh remove 후 ./scripts/gazebo_sim.sh --all
./scripts/demo_scenario.sh --upto 1 --attach
./scripts/demo_scenario.sh --from 2 --upto 4 --attach
./scripts/demo_scenario.sh --from 5 --upto 8 --attach shortcut

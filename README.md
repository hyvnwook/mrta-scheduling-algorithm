# MRTA (Multi-Robot Task Allocation) System

## Project Overview
본 프로젝트는 다중 로봇 작업 스케줄링(Multi Robot Task Scheduling) 문제를 해결하기 위한 알고리즘을 개발한 결과물입니다.
시야 범위가 넓은 드론(Drone)을 활용해 미지(UNKNOWN)의 영역과 작업을 탐색하고, 휠(Wheel)과 캐터필러(Caterpillar) 로봇이 발견된 작업을 수행하도록 역할을 분담하여 시스템의 효율성을 극대화했습니다.

## Development Environment
- **Language:** C++17
- **Compiler:** MSVC C++17 (19.40.33811)
- **Build System:** MSBuild (17.10.4.21802)
- **OS:** Windows 10 (x86)

## Core Algorithms

### 1. Drone Map Search Algorithm (탐색 알고리즘)
드론이 맵을 최대한 넓고 효율적으로 탐색하도록 **개미군집 알고리즘(Ant Colony Optimization)** 의 페로몬 아이디어를 차용했습니다.
- **Pheromone Map:** 로봇이 이동한 경로에 페로몬을 남기며, 이 페로몬은 시간이 지남에 따라 서서히 감소(decay)합니다. 드론은 페로몬 농도가 가장 낮은 미탐색 영역을 우선적으로 방문합니다.
- **Density-based Targeting:** 국소적인 페로몬 농도만 비교할 경우 벽에 막혀 제자리걸음(Toggle)을 하는 문제를 해결하기 위해, 5x5 크기의 '페로몬 밀도(Pheromone density) 맵'을 도입하여 타겟팅의 정확도를 높였습니다.
- **Dijkstra Routing:** 설정된 타겟으로 이동할 때 최소 페로몬 코스트를 가지는 다익스트라(Dijkstra) 알고리즘을 사용하여 중복 탐색을 방지했습니다.

### 2. Robot Task Assign Algorithm (작업 할당 알고리즘)
지속적으로 발생하는 작업(Task)들을 한정된 로봇에게 최적으로 분배하기 위해 실시간 환경에 맞춘 하이브리드 스케줄링 방식을 설계했습니다.
- **초기 할당 (Hungarian Algorithm):** 최초 8개의 작업이 모일 때까지 대기한 후, 헝가리안 알고리즘을 사용해 4대의 로봇에게 작업을 일괄 할당합니다.
- **추가 할당 (Suffrage Algorithm):** 이후 추가되는 작업들은 로봇별 이동 및 처리 코스트(다익스트라 기반)를 계산한 뒤, 1순위와 2순위 로봇 간의 비용 차이가 큰 작업부터 우선적으로 배정하는 Suffrage 알고리즘 방식을 적용했습니다.

## Simulation Results
총 30회의 반복 실험(시뮬레이션 당 최대 작업 수 16개)을 통해 제안한 알고리즘의 성능을 검증했습니다.
- **평균 탐지율:** 9.46개 (약 59.1%)
- **평균 수행 완료율:** 5.96개 (약 37.2%)
- **최고 성능 (Best Case):** 13개 작업 완료

결과적으로 본 알고리즘은 제한된 자원(에너지)과 불완전한 정보 환경 속에서도 탐색과 작업 분배의 균형을 훌륭하게 유지하며 안정적인 작업 처리량을 보였습니다.

## Contributors
- **이현욱 (@hyvnwook)**
- **이상엽**
- **최원우**

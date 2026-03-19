#include "schedular.h"
#include <queue>
#include <map>
#include <algorithm>
#include <unordered_set>

using namespace std;

vector<vector<int>> pheromone_map;
vector<vector<int>> density_map;
const int MAX_PHEROMONE = 100;
const int EDGE_PHEROMONE = 30;
const int PHEROMONE_DECAY = 1;
const int INF = 0x3f3f3f3f;
vector<Coord> drone_target(2, { -1, -1 });
vector<int> robotTaskMap;
bool initialAssigned = false;

bool boundry_check(int x, int y, int map_size) {
    if (x < 2 || x >= map_size - 2 || y < 2 || y >= map_size - 2) return false;
    return true;
}

void initialize_pheromone_map(int map_size) {
    pheromone_map = vector<vector<int>>(map_size, vector<int>(map_size, 0));
    for (int x = 0; x < map_size; ++x) {
        pheromone_map[x][0] = EDGE_PHEROMONE;
        pheromone_map[x][map_size - 1] = EDGE_PHEROMONE;
    }
    for (int y = 0; y < map_size; ++y) {
        pheromone_map[0][y] = EDGE_PHEROMONE;
        pheromone_map[map_size - 1][y] = EDGE_PHEROMONE;
    }
}

void safe_set_pheromone(int x, int y, int map_size, int value) {
    if (x >= 0 && x < map_size && y >= 0 && y < map_size) {
        pheromone_map[x][y] = min(INF, pheromone_map[x][y] + value);
    }
}

void initialize_pheromone_density_map(int map_size) {
    density_map = vector<vector<int>>(map_size, vector<int>(map_size, 0));
}

void update_pheromone_density_map(const vector<shared_ptr<ROBOT>>& robots, const vector<vector<OBJECT>>& known_object_map, int map_size) {

    for (int x = 0; x < map_size; ++x)
        for (int y = 0; y < map_size; ++y) {
            int sum = 0, cnt = 0;
            for (int dx = -2; dx <= 2; ++dx) {
                for (int dy = -2; dy <= 2; ++dy) {
                    int nx = x + dx, ny = y + dy;
                    if (nx >= 0 && nx < map_size && ny >= 0 && ny < map_size && known_object_map[nx][ny] != OBJECT::WALL) {
                        sum = sum + pheromone_map[nx][ny];
                        cnt++;
                    }
                }
            }
            density_map[x][y] = (cnt > 0) ? (sum / cnt) : INF;
        }
}

ROBOT::ACTION BFS_to_boundry(const Coord& start,
    const vector<vector<OBJECT>>& known_object_map,
    int map_size)
{
    if (boundry_check(start.x, start.y, map_size))
        return ROBOT::ACTION::HOLD;

    constexpr int dx[4] = { 1, -1,  0,  0 };
    constexpr int dy[4] = { 0,  0,  1, -1 };
    constexpr ROBOT::ACTION act[4] = {
        ROBOT::ACTION::RIGHT, ROBOT::ACTION::LEFT,
        ROBOT::ACTION::UP,    ROBOT::ACTION::DOWN
    };

    vector<vector<bool>> vis(map_size, vector<bool>(map_size, false));
    vector<vector<pair<int, int>>> prv(map_size,
        vector<pair<int, int>>(map_size, { -1,-1 }));

    queue<Coord> q;
    q.push(start); vis[start.x][start.y] = true;
    Coord goal = start;  bool found = false;

    while (!q.empty() && !found) {
        Coord u = q.front(); q.pop();
        for (int k = 0; k < 4; ++k) {
            int nx = u.x + dx[k], ny = u.y + dy[k];
            if (nx < 0 || ny < 0 || nx >= map_size || ny >= map_size) continue;
            if (vis[nx][ny] || known_object_map[nx][ny] == OBJECT::WALL)     continue;

            vis[nx][ny] = true;  prv[nx][ny] = { u.x, u.y };

            if (boundry_check(nx, ny, map_size)) {
                goal = { nx, ny };
                found = true;
                break;
            }
            q.push({ nx, ny });
        }
    }

    if (!found) return ROBOT::ACTION::HOLD;

    Coord cur = goal;
    while (prv[cur.x][cur.y] != make_pair(start.x, start.y))
        cur = { prv[cur.x][cur.y].first, prv[cur.x][cur.y].second };

    if (cur.x == start.x + 1) return ROBOT::ACTION::RIGHT;
    else if (cur.x == start.x - 1) return ROBOT::ACTION::LEFT;
    else if (cur.y == start.y + 1) return ROBOT::ACTION::UP;
    else if (cur.y == start.y - 1) return ROBOT::ACTION::DOWN;
    return ROBOT::ACTION::HOLD;
}

void update_pheromone_map(const vector<shared_ptr<ROBOT>>& robots, const vector<vector<OBJECT>>& known_object_map, int map_size) {
    for (int x = 0; x < map_size; ++x)
        for (int y = 0; y < map_size; ++y) {
            if (known_object_map[x][y] == OBJECT::WALL) {
                pheromone_map[x][y] = INF;
                continue;
            }
            if ((known_object_map[x][y] != OBJECT::WALL) && (known_object_map[x][y] != OBJECT::UNKNOWN)) {
                pheromone_map[x][y] = max(1, pheromone_map[x][y] - PHEROMONE_DECAY);
            }
        }

    for (const auto& robot : robots) {
        Coord pos = robot->get_coord();
        int M = pheromone_map.size();
        if (robot->type == ROBOT::TYPE::DRONE) {
            safe_set_pheromone(pos.x, pos.y, M, MAX_PHEROMONE);

            safe_set_pheromone(pos.x + 1, pos.y, M, 80);
            safe_set_pheromone(pos.x - 1, pos.y, M, 80);
            safe_set_pheromone(pos.x, pos.y + 1, M, 80);
            safe_set_pheromone(pos.x, pos.y - 1, M, 80);

            safe_set_pheromone(pos.x - 1, pos.y + 1, M, 70);
            safe_set_pheromone(pos.x + 1, pos.y + 1, M, 70);
            safe_set_pheromone(pos.x - 1, pos.y - 1, M, 70);
            safe_set_pheromone(pos.x + 1, pos.y - 1, M, 70);

            safe_set_pheromone(pos.x, pos.y + 2, M, 60);
            safe_set_pheromone(pos.x, pos.y - 2, M, 60);
            safe_set_pheromone(pos.x + 2, pos.y, M, 60);
            safe_set_pheromone(pos.x - 2, pos.y, M, 60);

            safe_set_pheromone(pos.x + 1, pos.y + 2, M, 50);
            safe_set_pheromone(pos.x + 1, pos.y - 2, M, 50);
            safe_set_pheromone(pos.x - 1, pos.y + 2, M, 50);
            safe_set_pheromone(pos.x - 1, pos.y - 2, M, 50);
            safe_set_pheromone(pos.x + 2, pos.y + 1, M, 50);
            safe_set_pheromone(pos.x + 2, pos.y - 1, M, 50);
            safe_set_pheromone(pos.x - 2, pos.y + 1, M, 50);
            safe_set_pheromone(pos.x - 2, pos.y - 1, M, 50);

            safe_set_pheromone(pos.x + 2, pos.y + 2, M, 40);
            safe_set_pheromone(pos.x + 2, pos.y - 2, M, 40);
            safe_set_pheromone(pos.x - 2, pos.y + 2, M, 40);
            safe_set_pheromone(pos.x - 2, pos.y - 2, M, 40);
        }
        if (robot->type == ROBOT::TYPE::CATERPILLAR) {
            safe_set_pheromone(pos.x, pos.y, M, MAX_PHEROMONE);

            safe_set_pheromone(pos.x + 1, pos.y, M, 80);
            safe_set_pheromone(pos.x - 1, pos.y, M, 80);
            safe_set_pheromone(pos.x, pos.y + 1, M, 80);
            safe_set_pheromone(pos.x, pos.y - 1, M, 80);

            safe_set_pheromone(pos.x - 1, pos.y + 1, M, 70);
            safe_set_pheromone(pos.x + 1, pos.y + 1, M, 70);
            safe_set_pheromone(pos.x - 1, pos.y - 1, M, 70);
            safe_set_pheromone(pos.x + 1, pos.y - 1, M, 70);
        }
        if (robot->type == ROBOT::TYPE::WHEEL) {
            safe_set_pheromone(pos.x, pos.y, M, MAX_PHEROMONE);

            safe_set_pheromone(pos.x + 1, pos.y, M, 80);
            safe_set_pheromone(pos.x - 1, pos.y, M, 80);
            safe_set_pheromone(pos.x, pos.y + 1, M, 80);
            safe_set_pheromone(pos.x, pos.y - 1, M, 80);
        }
    }
}

bool nooverlap_check(const Coord& a, const Coord& b, int map_size) {
    int gap = map_size / 5;
    return (abs(a.x - b.x) > gap || abs(a.y - b.y) > gap);
}

Coord target_lockon(const vector<vector<int>>& density_map, const Coord& here, const vector<Coord>& other_targets) {
    int N = density_map.size();
    int LAMBDA = 50;
    int min_val = INF;
    vector<Coord> min_coords;

    for (int x = 0; x < N; ++x) {
        for (int y = 0; y < N; ++y) {
            if (density_map[x][y] < min_val) {
                if (density_map[x][y] == INF) continue;
                min_val = density_map[x][y];
                min_coords.clear();
                min_coords.push_back({ x, y });
            }
            else if (density_map[x][y] == min_val) {
                min_coords.push_back({ x, y });
            }
        }
    }

    Coord best_coord = here;
    double best_score = INF;
    for (const Coord& pos : min_coords) {
        if (pos == here) continue;
        int d_min = INF;
        for (const Coord& tgt : other_targets)
            if (tgt.x >= 0)
                d_min = min(d_min, abs(pos.x - tgt.x) + abs(pos.y - tgt.y));
        if (d_min == INF) d_min = N * 2;

        double score = density_map[pos.x][pos.y] +
            static_cast<double>(LAMBDA) / (d_min + 1);

        if (score < best_score) {
            best_score = score;
            best_coord = pos;
        }
    }
    return best_coord;
}

vector<vector<int>> build_cost_map_by_type(
    const vector<vector<vector<int>>>& known_cost_map,
    ROBOT::TYPE type)
{
    int N = known_cost_map.size();
    vector<vector<int>> m(N, vector<int>(N, INF));
    size_t idx = static_cast<size_t>(type);

    for (int x = 0; x < N; ++x)
        for (int y = 0; y < N; ++y) {
            m[x][y] = known_cost_map[x][y][idx];
        }
    return m;
}

inline int add_cost(int a, int b) {
    long long s = (long long)a + b;
    return (s >= INF) ? INF : (int)s;
}

vector<Coord> pheromone_dijkstra(const vector<vector<int>>& grid,
    const vector<vector<OBJECT>>& known_object_map,
    const Coord& start,
    const Coord& goal,
    bool useboundary)
{
    const int N = grid.size();
    static const int dx[4] = { 1,-1,0,0 };
    static const int dy[4] = { 0,0,1,-1 };

    vector<vector<int>> dist(N, vector<int>(N, INF));
    vector<vector<Coord>> parent(N, vector<Coord>(N, { -1,-1 }));
    using QNode = pair<int, Coord>;
    priority_queue<QNode, vector<QNode>, greater<QNode>> pq;

    dist[start.x][start.y] = 0;
    pq.push({ 0,start });

    while (!pq.empty()) {
        auto [cost, u] = pq.top();pq.pop();
        if (cost != dist[u.x][u.y]) continue;
        if (u == goal) break;
        for (int dir = 0;dir < 4;++dir) {
            int nx = u.x + dx[dir];
            int ny = u.y + dy[dir];
            int w;
            if (nx < 0 || ny < 0 || nx >= N || ny >= N) continue;
            if (known_object_map[nx][ny] == OBJECT::WALL) continue;
            if (useboundary && !boundry_check(nx, ny, N)) continue;
            if (useboundary) {
                w = max(1, grid[nx][ny]);
            }
            else {
                w = grid[nx][ny];
            }
            if (w <= 0 || w == INF) continue;
            int ncost = add_cost(cost, w);
            if (ncost < dist[nx][ny]) {
                dist[nx][ny] = ncost;
                parent[nx][ny] = u;
                pq.push({ ncost, {nx, ny} });
            }
        }
    }

    if (dist[goal.x][goal.y] == INF) return {};
    vector<Coord> path;
    for (Coord cur = goal; cur != Coord{ -1,-1 }; cur = parent[cur.x][cur.y]) {
        path.push_back(cur);
        if (cur == start) break;
    }
    reverse(path.begin(), path.end());
    return path;
}

ROBOT::ACTION Move_to_target(const vector<shared_ptr<ROBOT>>& robots,
    const ROBOT& robot,
    Coord start,
    Coord target,
    const vector<vector<OBJECT>>& known_object_map,
    const vector<vector<int>>& pheromone_map,
    const vector<vector<vector<int>>>& known_cost_map)
{
    vector<Coord> path;
    if (start == target) return ROBOT::ACTION::HOLD;

    if (robot.type == ROBOT::TYPE::DRONE) {
        path = pheromone_dijkstra(pheromone_map, known_object_map, start, target, true);
    }
    else if (robot.type == ROBOT::TYPE::CATERPILLAR) {
        auto CAT_Costmap = build_cost_map_by_type(known_cost_map, robot.type);
        path = pheromone_dijkstra(CAT_Costmap, known_object_map, start, target, false);
    }
    else if (robot.type == ROBOT::TYPE::WHEEL) {
        auto WHE_Costmap = build_cost_map_by_type(known_cost_map, robot.type);
        path = pheromone_dijkstra(WHE_Costmap, known_object_map, start, target, false);
    }

    if (path.size() < 2) {
        return ROBOT::ACTION::HOLD;
    }

    Coord next = path[1];
    int dx = next.x - start.x;
    int dy = next.y - start.y;
    if (dx == 1)  return ROBOT::ACTION::RIGHT;
    if (dx == -1) return ROBOT::ACTION::LEFT;
    if (dy == 1)  return ROBOT::ACTION::UP;
    if (dy == -1) return ROBOT::ACTION::DOWN;
    return ROBOT::ACTION::HOLD;
}

int dijkstraCost(Coord start, Coord goal,
    const vector<vector<vector<int>>>& known_cost_map,
    const vector<vector<OBJECT>>& known_object_map,
    ROBOT::TYPE type)
{
    const int N = known_cost_map.size();
    vector<vector<int>> dist(N, vector<int>(N, INF));
    priority_queue<tuple<int, int, int>, vector<tuple<int, int, int>>, greater<>> pq;
    size_t idx = static_cast<size_t>(type);

    auto is_valid = [&](int x, int y) {
        return 0 <= x && x < N && 0 <= y && y < N
            && known_object_map[x][y] != OBJECT::WALL
            && known_cost_map[x][y][idx] >= 0;
        };

    dist[start.x][start.y] = 0;
    pq.emplace(0, start.x, start.y);

    const int dx[] = { 1, -1, 0, 0 };
    const int dy[] = { 0, 0, 1, -1 };

    while (!pq.empty()) {
        auto [cost, x, y] = pq.top(); pq.pop();
        if (Coord(x, y) == goal) return cost;
        if (cost > dist[x][y]) continue;

        for (int d = 0; d < 4; ++d) {
            int nx = x + dx[d];
            int ny = y + dy[d];
            if (!is_valid(nx, ny)) continue;

            int step_cost = known_cost_map[nx][ny][idx];
            if (step_cost < 0 || step_cost == INF) continue;
            int new_cost = min(INF, cost + step_cost);
            if (new_cost < dist[nx][ny]) {
                dist[nx][ny] = new_cost;
                pq.emplace(new_cost, nx, ny);
            }
        }
    }

    return INF;
}

vector<int> hungarianSolve(const vector<vector<int>>& cost) {
    int n = cost.size();
    vector<int> u(n + 1), v(n + 1), p(n + 1), way(n + 1);

    for (int i = 1; i <= n; ++i) {
        p[0] = i;
        int j0 = 0;
        vector<int> minv(n + 1, numeric_limits<int>::max());
        vector<bool> used(n + 1, false);

        do {
            used[j0] = true;
            int i0 = p[j0], delta = numeric_limits<int>::max(), j1 = 0;
            for (int j = 1; j <= n; ++j) {
                if (used[j]) continue;
                int cur = cost[i0 - 1][j - 1] - u[i0] - v[j];
                if (cur < minv[j]) {
                    minv[j] = cur;
                    way[j] = j0;
                }
                if (minv[j] < delta) {
                    delta = minv[j];
                    j1 = j;
                }
            }
            for (int j = 0; j <= n; ++j) {
                if (used[j]) {
                    u[p[j]] += delta;
                    v[j] -= delta;
                }
                else {
                    minv[j] -= delta;
                }
            }
            j0 = j1;
        } while (p[j0] != 0);

        do {
            int j1 = way[j0];
            p[j0] = p[j1];
            j0 = j1;
        } while (j0);
    }

    vector<int> result(n);
    for (int j = 1; j <= n; ++j)
        if (p[j] > 0)
            result[p[j] - 1] = j - 1;

    return result;
}

void assignInitialTasksHungarian(
    const vector<shared_ptr<ROBOT>>& robots,
    const vector<shared_ptr<TASK>>& active_tasks,
    const vector<vector<vector<int>>>& known_cost_map,
    const vector<vector<OBJECT>>& known_object_map,
    vector<int>& robotTaskMap
) {
    int R = robots.size();
    int T = active_tasks.size();
    int N = max(R, T);

    vector<vector<int>> cost(N, vector<int>(N, 999999));

    for (int i = 0; i < R; ++i) {
        Coord from = robots[i]->get_coord();
        for (int j = 0; j < T; ++j) {
            Coord to = active_tasks[j]->coord;
            int moveCost = dijkstraCost(from, to, known_cost_map, known_object_map, robots[i]->type);
            int taskCost = active_tasks[j]->get_cost(robots[i]->type);
            cost[i][j] = moveCost + taskCost;
        }
    }

    vector<int> assignment = hungarianSolve(cost);

    for (int i = 0; i < R; ++i) {
        int col = assignment[i];
        robotTaskMap[robots[i]->id] = (col < T) ? col : -1;
    }
}

void assignAdditionalTasksSufferage(const vector<shared_ptr<ROBOT>>& robots,
    const vector<shared_ptr<TASK>>& active_tasks,
    const vector<vector<vector<int>>>& known_cost_map,
    const vector<vector<OBJECT>>& known_object_map,
    vector<int>& robotTaskMap)
{
    unordered_set<int> reserved;
    for (int tid : robotTaskMap)
        if (tid >= 0) reserved.insert(tid);

    vector<int> freeRobotIdx;
    for (size_t i = 0;i < robots.size();++i) {
        const auto& r = robots[i];
        if (r->get_status() == ROBOT::STATUS::IDLE && r->get_energy() > 0) {
            if (robotTaskMap.size() <= r->id || robotTaskMap[r->id] == -1)
                freeRobotIdx.push_back((int)i);
        }
    }

    vector<int> unassignedTaskIdx;
    for (size_t t = 0;t < active_tasks.size();++t) {
        if (active_tasks[t]->get_assigned_robot_id() != -1) continue;
        if (reserved.count(int(t))) continue;
        unassignedTaskIdx.push_back((int)t);
    }

    if (freeRobotIdx.empty() || unassignedTaskIdx.empty()) return;

    while (!freeRobotIdx.empty() && !unassignedTaskIdx.empty()) {
        struct Info { int taskIdx; int bestRobotIdx; int best; int second; };
        vector<Info> infos;
        infos.reserve(unassignedTaskIdx.size());

        for (int tIdx : unassignedTaskIdx) {
            int best = INF, second = INF, bestRobot = -1;
            for (int rVecIdx : freeRobotIdx) {
                auto& robot = robots[rVecIdx];
                int move = dijkstraCost(robot->get_coord(), active_tasks[tIdx]->coord, known_cost_map, known_object_map, robot->type);
                int cost = move + active_tasks[tIdx]->get_cost(robot->type);
                if (cost < best) {
                    second = best; best = cost; bestRobot = rVecIdx;
                }
                else if (cost < second) {
                    second = cost;
                }
            }
            if (bestRobot != -1) {
                infos.push_back({ tIdx, bestRobot, best, second });
            }
        }
        if (infos.empty()) break;

        auto it = max_element(infos.begin(), infos.end(), [](const Info& a, const Info& b) {
            return (a.second == INF ? -1 : a.second - a.best) < (b.second == INF ? -1 : b.second - b.best);
            });
        Info chosen = *it;

        int robotGlobalId = robots[chosen.bestRobotIdx]->id;
        if (robotTaskMap.size() <= robotGlobalId) robotTaskMap.resize(robotGlobalId + 1, -1);
        robotTaskMap[robotGlobalId] = chosen.taskIdx;

        freeRobotIdx.erase(remove(freeRobotIdx.begin(), freeRobotIdx.end(), chosen.bestRobotIdx), freeRobotIdx.end());
        unassignedTaskIdx.erase(remove(unassignedTaskIdx.begin(), unassignedTaskIdx.end(), chosen.taskIdx), unassignedTaskIdx.end());
    }
}

void Scheduler::on_info_updated(const set<Coord>& observed_coords,
    const set<Coord>& updated_coords,
    const vector<vector<vector<int>>>& known_cost_map,
    const vector<vector<OBJECT>>& known_object_map,
    const vector<shared_ptr<TASK>>& active_tasks,
    const vector<shared_ptr<ROBOT>>& robots)
{
    if (pheromone_map.empty()) initialize_pheromone_map(known_object_map.size());
    int map_size = known_object_map.size();
    update_pheromone_map(robots, known_object_map, map_size);
    if (density_map.empty()) initialize_pheromone_density_map(map_size);
    update_pheromone_density_map(robots, known_object_map, map_size);

    int max_robot_id = 0;
    for (const auto& r : robots)
        max_robot_id = std::max(max_robot_id, r->id);

    if (robotTaskMap.size() <= max_robot_id) {
        robotTaskMap.resize(max_robot_id + 1, -1);
    }

    if (!initialAssigned && active_tasks.size() >= 5) {
        assignInitialTasksHungarian(
            robots,
            active_tasks,
            known_cost_map,
            known_object_map,
            robotTaskMap
        );
        initialAssigned = true;
    }
    if (initialAssigned) assignAdditionalTasksSufferage(robots, active_tasks, known_cost_map, known_object_map, robotTaskMap);
}

bool Scheduler::on_task_reached(const set<Coord>& observed_coords,
    const set<Coord>& updated_coords,
    const vector<vector<vector<int>>>& known_cost_map,
    const vector<vector<OBJECT>>& known_object_map,
    const vector<shared_ptr<TASK>>& active_tasks,
    const vector<shared_ptr<ROBOT>>& robots,
    const ROBOT& robot,
    const TASK& task)
{
    if (robot.type == ROBOT::TYPE::DRONE) return false;
    if (robot.get_energy() < task.get_cost(robot.type)) return false;

    if (task.get_assigned_robot_id() != -1) return false;
    robotTaskMap[robot.id] = -1;
    return true;
}

ROBOT::ACTION Scheduler::idle_action(const set<Coord>& observed_coords,
    const set<Coord>& updated_coords,
    const vector<vector<vector<int>>>& known_cost_map,
    const vector<vector<OBJECT>>& known_object_map,
    const vector<shared_ptr<TASK>>& active_tasks,
    const vector<shared_ptr<ROBOT>>& robots,
    const ROBOT& robot)
{
    int N = known_object_map.size();
    int unknown_cnt = 0;
    for (int x = 0; x < N; x++) {
        for (int y = 0; y < N; y++) {
            if (known_object_map[x][y] == OBJECT::UNKNOWN) {
                unknown_cnt++;
            }
        }
    }
    double unknown_rate = static_cast<double>(unknown_cnt) / (N * N);

    if (robot.type == ROBOT::TYPE::DRONE) {
        if (active_tasks.size() >= 6 && unknown_rate <= 0.2) {
            return ROBOT::ACTION::HOLD;
        }
        else {
            Coord here = robot.get_coord();

            if (!boundry_check(here.x, here.y, N)) {
                return BFS_to_boundry(here, known_object_map, N);
            }

            int idx = (robot.id == 0) ? 0 : 1;

            Coord& my_target = drone_target[idx];
            Coord other_target = drone_target[1 - idx];

            vector<Coord> other_targets;
            if (other_target.x >= 0 && other_target.y >= 0)
                other_targets.push_back(other_target);

            if (my_target.x < 0 && my_target.y < 0) {
                my_target = target_lockon(density_map, here, other_targets);
            }
            if ((here == my_target) || known_object_map[my_target.x][my_target.y] == OBJECT::WALL) {
                my_target = target_lockon(density_map, here, other_targets);
            }
            auto path = pheromone_dijkstra(pheromone_map, known_object_map, here, my_target, true);
            if (path.size() < 2) {
                my_target = target_lockon(density_map, here, other_targets);
            }

            return Move_to_target(robots, robot, here, my_target, known_object_map, pheromone_map, known_cost_map);
        }
    }

    if (robot.type != ROBOT::TYPE::DRONE) {
        int id = robot.id;
        if (robotTaskMap.size() <= id || robotTaskMap[id] == -1)
            return ROBOT::ACTION::HOLD;

        int task_idx = robotTaskMap[id];
        if (task_idx >= active_tasks.size())
            return ROBOT::ACTION::HOLD;

        Coord here = robot.get_coord();
        Coord goal = active_tasks[task_idx]->coord;
        return Move_to_target(robots, robot, here, goal, known_object_map, pheromone_map, known_cost_map);
    };
    return ROBOT::ACTION::HOLD;
}
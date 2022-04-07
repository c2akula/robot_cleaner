#include <string>
#include <vector>
#include <algorithm>
#include <optional>
#include <cstdio>

using Grid = std::vector<std::string>;

enum class Direction
{
        R, D, L, U, Count
};

constexpr auto as_int(const Direction& d)
{
        return static_cast<int>(d);
}

/**
 * @brief Map provides a thin wrapper over a Grid object to conveniently access its contents.
 */
class Map
{
        const Grid& grid;
        const int w;
        const int h;

    public:
        struct Position
        {
                int x, y;
        };

    public:
        explicit Map(const Grid& grid) : grid{grid}, w{static_cast<int>(grid.front().size())},
                                         h{static_cast<int>(grid.size())}
        {
        }

        /**
         * @brief Obtains the value of the cell in the map at the given position.
         * @param p Position of the cell whose value is requested.
         * @return value of the cell if position is within bounds else std::nullopt.
         */
        constexpr auto operator()(const Position p) const -> std::optional<int>
        {
                if (const auto[x, y] = p; (x < w && x >= 0) && (y < h && y >= 0)) {
                        return {grid[y][x]};
                }
                return {};
        }

        /**
         * @brief Computes the no. of cells in the map.
         */
        [[nodiscard]]
        auto size() const -> size_t
        {
                return w * h;
        }
};

constexpr auto operator+(const Map::Position& lhs, const Direction& rhs) -> Map::Position
{
        constexpr int INC_DIRECTIONS[as_int(Direction::Count)]{
                [as_int(Direction::R)] = 1,
                [as_int(Direction::D)] = 1,
                [as_int(Direction::L)] = -1,
                [as_int(Direction::U)] = -1,
        };

        const auto d = INC_DIRECTIONS[as_int(rhs)];
        const auto[x, y] = lhs;
        if (rhs == Direction::R || rhs == Direction::L) {
                return {x + d, y};
        }
        return {x, y + d};
}

constexpr auto operator==(const Map::Position& lhs, const Map::Position& rhs) -> bool
{
        return (lhs.x == rhs.x) && (lhs.y == rhs.y);
}

/**
 * @brief A cleaning robot that moves through the given Map to clean as many cells as possible. The run() method is the
 * main control loop of the robot which terminates when the robot cannot make progress and returns the no. of clean cells
 * at this time.
 */
class Robot
{
        static constexpr Direction PEEK_DIRECTIONS[as_int(Direction::Count)]{
                [as_int(Direction::R)] = Direction::R,
                [as_int(Direction::D)] = Direction::D,
                [as_int(Direction::L)] = Direction::L,
                [as_int(Direction::U)] = Direction::U,
        };

    public:
        enum class PeekStatus
        {
                Ok,
                Visited,
                Blocked,
        };

    private:
        const Map      & map;
        const Direction* dir;
        Map::Position              pos;
        std::vector<Map::Position> visited;


    public:
        explicit Robot(const Map& map) : dir{std::begin(PEEK_DIRECTIONS)}, pos{}, map{map}
        {
                visited.reserve(map.size());
                visited.push_back(pos);
        }

    public:
        /**
         * @brief Looks ahead in the given direction.
         * @param d Direction to peek
         * @return new position if not visited or visited and current position if blocked.
         */
        auto peek(Direction d) -> std::pair<Map::Position, PeekStatus>
        {
                if (const auto _pos = pos + d; map(_pos) == '.') {
                        if (!is_visited(_pos)) {
                                return {_pos, PeekStatus::Ok};
                        }

                        return {_pos, PeekStatus::Visited};
                }
                return {pos, PeekStatus::Blocked};
        }

        /**
         * @brief Updates the current position of the robot to the target.
         * @param tgt Position to move to.
         * @return updated position.
         */
        auto move_to(Map::Position tgt) -> Map::Position
        {
                pos = tgt;
                return pos;
        }

        /**
         * @brief Change the heading of the robot.
         */
        auto change_direction()
        {
                dir++;
                if (dir == std::end(PEEK_DIRECTIONS)) {
                        dir = std::begin(PEEK_DIRECTIONS);
                }
        }

        /**
         * @brief Main loop that moves the robot through the map. Terminates when the robot is unable to make progress.
         * @return no. of cells cleaned in the map.
         */
        auto run() -> size_t
        {
                auto just_visited = false;
                auto nblocked     = 0;
                do {
                        const auto[p, status] = peek(*dir);

                        if (status == PeekStatus::Visited) {
                                nblocked = 0;
                                // move one step ahead and peek again
                                if (!just_visited) {
                                        move_to(p);
                                        just_visited = true;
                                }
                                else {
                                        break;
                                }
                        }
                        else if (status == PeekStatus::Ok) {
                                nblocked = 0;
                                if (just_visited) {
                                        just_visited = false;
                                }
                                visited.push_back(move_to(p));
                        }
                        else {
                                nblocked += 1;
                                if (nblocked == as_int(Direction::Count)) {
                                        // we're blocked in all directions
                                        break;
                                }
                                change_direction();
                        }
                }
                while (true);

                return visited.size();
        }

    private:
        [[nodiscard]]
        auto is_visited(const Map::Position& p) const -> bool
        {
                return std::any_of(visited.begin(), visited.end(), [p](const auto& v) {
                    return v == p;
                });
        }
};

auto main() -> int
{
        struct TestStruct
        {
                Map map;
                int ncleaned;
        };

        std::vector<TestStruct> Tests{
                TestStruct{Map{Grid{"....x..", "x......", ".....x.", "......."}}, 15},
                TestStruct{Map{Grid{"...x..", "....xx", "..x..."}}, 6},
                TestStruct{Map{Grid{"...x.", ".x..x", "x...x", "..x.."}}, 9},
                TestStruct{Map{Grid{".", "."}}, 2},
                TestStruct{Map{Grid{".x"}}, 1},
                TestStruct{Map{Grid{".", "x"}}, 1},
        };

        std::for_each(Tests.begin(), Tests.end(), [i = 0](const auto& exp) mutable {
            Robot robot{exp.map};
            std::printf("test [%d]: ", i);
            if (const auto got_ncleaned = robot.run() != exp.ncleaned) {
                    std::printf("FAIL. exp: %d, got: %d\n", exp.ncleaned, got_ncleaned);
            }
            else {
                    std::printf("OK\n");
            }
            i++;
        });
}
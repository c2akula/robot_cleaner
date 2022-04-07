#include <string>
#include <vector>
#include <algorithm>
#include <optional>
#include <iostream>

using Grid = std::vector<std::string>;

enum class Direction
{
        R, D, L, U, Count
};

constexpr auto as_int(const Direction& d)
{
        return static_cast<int>(d);
}

constexpr Direction PEEK_DIRECTIONS[as_int(Direction::Count)]{
        [as_int(Direction::R)] = Direction::R,
        [as_int(Direction::D)] = Direction::D,
        [as_int(Direction::L)] = Direction::L,
        [as_int(Direction::U)] = Direction::U,
};

constexpr int INC_DIRECTIONS[as_int(Direction::Count)]{
        [as_int(Direction::R)] = 1,
        [as_int(Direction::D)] = 1,
        [as_int(Direction::L)] = -1,
        [as_int(Direction::U)] = -1,
};

struct Position
{
        int x{}, y{};

        constexpr auto operator+(Direction d) const -> Position
        {
                const auto _d = as_int(d);
                if (d == Direction::R || d == Direction::L) {
                        return {x + INC_DIRECTIONS[_d], y};
                }
                return {x, y + INC_DIRECTIONS[_d]};
        }
};

constexpr auto operator==(const Position& lhs, const Position& rhs) -> bool
{
        return (lhs.x == rhs.x) && (lhs.y == rhs.y);
}

struct Map
{
        const Grid& grid;
        const int w;
        const int h;

        explicit Map(const Grid& grid) : grid{grid}, w{static_cast<int>(grid.front().size())},
                                         h{static_cast<int>(grid.size())}
        {
        }

        constexpr auto operator()(const int x, const int y) const -> std::optional<int>
        {
                if ((x < w && x >= 0) && (y < h && y >= 0)) {
                        return {grid[y][x]};
                }
                return {};
        }

        constexpr auto operator()(const Position p) const -> std::optional<int>
        {
                const auto[x, y] = p;
                return (*this)(x, y);
        }
};


class Robot
{
    public:
        enum class PeekStatus
        {
                Ok,
                Visited,
                Blocked,
        };

    private:
        Map map;
        const Direction* dir;
        Position              pos{};
        std::vector<Position> visited;


    public:
        explicit Robot(const Grid& grid) : dir{std::begin(PEEK_DIRECTIONS)}, map{grid}
        {
                visited.reserve(map.w * map.h);
                visited.push_back(pos);
        }

    public:
        auto peek(Direction d) -> std::pair<Position, PeekStatus>
        {
                const auto _pos = pos + d; // peek position
                if (map(_pos) == '.') {
                        if (!is_visited(_pos)) {
                                return {_pos, PeekStatus::Ok};
                        }

                        return {_pos, PeekStatus::Visited};
                }
                return {{}, PeekStatus::Blocked};
        }

        auto move_to(Position tgt) -> Position
        {
                pos = tgt;
                return pos;
        }

        auto change_direction()
        {
                dir++;
                if (dir == std::end(PEEK_DIRECTIONS)) {
                        dir = std::begin(PEEK_DIRECTIONS);
                }
        }

        auto run()
        {
                auto just_visited = false;
                do {
                        const auto[p, status] = peek(*dir);
                        if (status == PeekStatus::Visited) {
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
                                if (just_visited) {
                                        just_visited = false;
                                }
                                visited.push_back(move_to(p));
                        }
                        else {
                                change_direction();
                        }
                }
                while (true);

                return visited.size();
        }

    private:
        [[nodiscard]]
        auto is_visited(const Position& p) const -> bool
        {
                return std::any_of(visited.begin(), visited.end(), [p](const auto& v) {
                    return v == p;
                });
        }

};

auto main() -> int
{
//        const Grid g{"....x..", "x......", ".....x.", "......."}; // 15
//        const Grid g{"...x..", "....xx", "..x..."}; // 6
        const Grid g{"...x.", ".x..x", "x...x", "..x.."}; // 9

        Robot      robot{g};
        const auto ncleaned = robot.run();
        std::cout << "ncleaned: " << ncleaned << '\n';
}
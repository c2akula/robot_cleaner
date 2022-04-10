#include <string>
#include <vector>
#include <algorithm>
#include <optional>
#include <cstdio>
#include <variant>

/// Variant helper for using lambdas in-place
template <class... Ts>
struct visitor : Ts ... { using Ts::operator()...; };
template <class... Ts> visitor(Ts...) -> visitor<Ts...>;

/// Position
struct Position { int x, y; };

constexpr auto operator==(const Position& lhs, const Position& rhs) -> bool
{ return (lhs.x == rhs.x) && (lhs.y == rhs.y); }

/// Direction
template <int D>
struct Dir { static constexpr auto value = D; };

struct R : Dir<1> {}; // Right
struct D : Dir<1> {}; // Down
struct L : Dir<-1> {}; // Left
struct U : Dir<-1> {}; // Up

using Direction = std::variant<R, D, L, U>;

constexpr auto DirectionCount = std::variant_size_v<Direction>;

constexpr auto operator+(const Position& lhs, const Direction& rhs) -> Position
{
        return std::visit(visitor{
                [&lhs](const R&) -> Position { return {lhs.x + R::value, lhs.y}; },
                [&lhs](const D&) -> Position { return {lhs.x, lhs.y + D::value}; },
                [&lhs](const L&) -> Position { return {lhs.x + L::value, lhs.y}; },
                [&lhs](const U&) -> Position { return {lhs.x, lhs.y + U::value}; },
        }, rhs);
}

/// Cell States
struct Empty { Position pos; }; // Cell available to move into
struct Visited { Position pos; }; // Cell has been visited previously
struct Blocked {}; // Cell is occupied
using Cell = std::variant<Empty, Visited, Blocked>;

/**
 * @brief Represents the position and heading of the robot.
 */
struct Pose
{
        Position  p; // location
        Direction d; // heading

        [[nodiscard]]auto rotate() const
        {
                Pose pose{p, d};
                pose.d = std::visit(visitor{
                        [](const R&) -> Direction { return D{}; },
                        [](const D&) -> Direction { return L{}; },
                        [](const L&) -> Direction { return U{}; },
                        [](const U&) -> Direction { return R{}; },
                }, d);
                return pose;
        }

        [[nodiscard]]auto advance() const
        { return Pose{p + d, d}; }
};

using Poses = std::vector<Pose>;

/**
 * @brief Map provides a thin wrapper over a Grid object to conveniently access its contents.
 */
class Map
{
    public:
        using Layout = std::vector<std::string>;
        using Positions = std::vector<Position>;

    private:
        const Layout grid;
        int          w, h;
        Positions    visited;

    public:
        explicit Map(Layout g) : grid{std::move(g)}
        {
                w = static_cast<int>(grid.front().size());
                h = static_cast<int>(grid.size());
                visited.reserve(w * h);
                visited.push_back(Position{});
        }

        /**
         * @brief Obtains the value of the cell in the map at the given coordinate.
         * @param p Coordinate of the cell whose value is requested.
         * @return Empty if '.' or Blocked if 'x'.
         */
        auto operator()(const Position p) const -> Cell
        {
                const auto v = get_empty(p).value_or(Blocked{});
                return find_visited(p).value_or(v);
        }

        /**
         * @brief Add the given position to the visited list.
         */
        auto mark_visited(const Position p)
        { visited.push_back(p); }

        /**
         * @brief Computes the no. of cells in the map.
         */
        [[nodiscard]] auto count_visited() const -> size_t
        { return visited.size(); }

        auto show() const
        {
                for (const auto& s: grid) {
                        std::printf("[");
                        for (const auto& c: s) { std::printf(" %c ", c); }
                        std::printf("]\n");
                }
        }

        [[nodiscard]] auto shape() const -> std::pair<int, int>
        { return {w, h}; }

    private:
        [[nodiscard]] auto find_visited(const Position& p) const -> std::optional<Cell>
        {
                const auto pv = std::find(visited.begin(), visited.end(), p);
                if (pv == visited.end()) { return {}; }
                return Visited{p};
        }

        [[nodiscard]] auto get_empty(const Position& p) const -> std::optional<Cell>
        {
                const auto[x, y] = p;
                if ((x < w && x >= 0) && (y < h && y >= 0)) {
                        if (grid[y][x] == '.') { return Empty{p}; }
                }
                return {};
        }
};


/**
 * @brief A cleaning robot that moves through the given Map to clean as many cells as possible. The run() method is the
 * main control loop of the robot which terminates when the robot cannot make progress and returns the no. of clean cells
 * at this time.
 */
class Robot
{
        Map& map;
        bool  just_visited;
        int   nblocked;
        Poses poses;

    public:
        struct Running { Pose pose; };
        struct Stopped {};
        using State = std::variant<Stopped, Running>;

    public:
        explicit Robot(Map& map, const Pose pose) : map{map}, just_visited{}, nblocked{}
        {
                const auto[w, h] = map.shape();
                poses.reserve(w * h);
                poses.push_back(pose);
        }

    public:
        /**
         * @brief Scans the cell ahead of the robot in its current direction.
         */
        auto peek(const Pose pose) -> Cell
        { return map(pose.advance().p); }

        /**
         * @brief Moves the robot to the given cell.
         * @param tgt Cell to move into.
         * @return Running or Stopped.
         */
        auto move_to(const Cell& cell, const Pose& p) -> State
        {
                Pose pose = p;
                return std::visit(visitor{
                        [&](const Empty& e) -> State {
                            if (just_visited) { just_visited = false; }
                            nblocked = 0;
                            pose.p = e.pos;
                            map.mark_visited(pose.p);
                            poses.push_back(pose);
                            return Running{pose};
                        },
                        [&](const Visited& v) -> State {
                            if (just_visited) { return Stopped{}; }
                            just_visited = true;
                            nblocked     = 0;
                            pose.p = v.pos;
                            return Running{pose};
                        },
                        [&](const Blocked&) -> State {
                            pose = pose.rotate();
                            nblocked += 1;
                            if (nblocked == DirectionCount) { return Stopped{}; }
                            return Running{pose};
                        }
                }, cell);
        }

        /**
         * @brief Main loop that moves the robot through the map. Terminates when the robot is unable to make progress.
         * @return no. of cells cleaned in the map.
         */
        auto run() -> size_t
        {
                auto pose = poses.front(); // always current pose
                do {
                        const auto cell  = peek(pose);
                        const auto state = move_to(cell, pose);
                        if (std::holds_alternative<Stopped>(state)) { return poses.size(); }
                        pose = std::get<Running>(state).pose; // update pose
                }
                while (true);
        }

        auto show() const
        {
                const auto[w, h] = map.shape();

                Map::Layout m(static_cast<Map::Layout::size_type>(h),
                              std::string(static_cast<Map::Layout::size_type>(w), '\0'));

                for (const auto& pose: poses) {
                        const auto dc = std::visit(visitor{
                                [](const R&) { return 'r'; },
                                [](const D&) { return 'd'; },
                                [](const L&) { return 'l'; },
                                [](const U&) { return 'u'; },
                        }, pose.d);

                        const auto[x, y] = pose.p;
                        m[y][x] = dc;
                }

                Map _map{m};
                _map.show();
        }
};

auto main() -> int
{
        struct TestStruct
        {
                Map map;
                int ncleaned{};
        };

        std::vector<TestStruct> Tests{
                TestStruct{Map{{"....x..", "x......", ".....x.", "......."}}, 15},
                TestStruct{Map{{"...x..", "....xx", "..x..."}}, 6},
                TestStruct{Map{{"...x.", ".x..x", "x...x", "..x.."}}, 9},
                TestStruct{Map{{".", "."}}, 2},
                TestStruct{Map{{".x"}}, 1},
                TestStruct{Map{{".", "x"}}, 1},
        };

        std::for_each(Tests.begin(), Tests.end(), [i = 0](auto& exp) mutable {
            exp.map.show();
            Robot robot{exp.map, {Position{0, 0}, R{}}};
            std::printf("Path Traced: \n");
            const auto got_ncleaned = robot.run();
            robot.show();

            std::printf("test [%d]: ", i);
            if (got_ncleaned != exp.ncleaned) {
                    std::printf("FAIL. exp: %d, got: %d\n", exp.ncleaned, got_ncleaned);
            }
            else { std::printf("OK\n"); }
            i++;
        });
}
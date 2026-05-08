import java.util.*;

/**
 * This program finds an optimal synchronized schedule for two robots (A and B)
 * to reach their targets without colliding or swapping positions.
 */
public class RobotPlanner01 {

    // Map dimensions
    private int rows;
    private int cols;

    // Number of free cells (non-wall cells)
    private int freeCellsCount;

    // Mapping arrays: index -> coordinates
    private int[] rowOfIndex;
    private int[] colOfIndex;

    // Mapping array: coordinates -> index (-1 for walls)
    private int[][] indexOf;

    // Original map with cell types
    private char[][] cellType;

    // Robot positions (indices in free cell mapping)
    private int startA, startB, targetA, targetB;

    // Precomputed successors for each free cell
    private List<Integer>[] successors;

    // Direction vectors: N, E, S, W, WAIT
    private static final int[] DR = {-1, 0, 1, 0, 0};
    private static final int[] DC = {0, 1, 0, -1, 0};
    private static final String[] MOVE_NAMES = {"N", "E", "S", "W", "WAIT"};

    /**
     * Constructor: Parses the map and initializes all data structures
     * @param map Hospital map as array of strings
     */
    @SuppressWarnings("unchecked")
    public RobotPlanner01(String[] map) {
        this.rows = map.length;
        this.cols = map[0].length();
        this.cellType = new char[rows][cols];

        // First pass: Count free cells and store cell types
        freeCellsCount = 0;
        for (int r = 0; r < rows; r++) {
            for (int c = 0; c < cols; c++) {
                char cell = map[r].charAt(c);
                cellType[r][c] = cell;
                if (cell != '#') {
                    freeCellsCount++;
                }
            }
        }

        // Initialize mapping arrays
        rowOfIndex = new int[freeCellsCount];
        colOfIndex = new int[freeCellsCount];
        indexOf = new int[rows][cols];
        for (int r = 0; r < rows; r++) {
            Arrays.fill(indexOf[r], -1);
        }

        // Second pass: Assign indices to free cells and locate robots
        int idx = 0;
        startA = startB = targetA = targetB = -1;

        for (int r = 0; r < rows; r++) {
            for (int c = 0; c < cols; c++) {
                char cell = cellType[r][c];
                if (cell != '#') {
                    rowOfIndex[idx] = r;
                    colOfIndex[idx] = c;
                    indexOf[r][c] = idx;

                    // Record robot positions
                    if (cell == 'A') startA = idx;
                    else if (cell == 'a') targetA = idx;
                    else if (cell == 'B') startB = idx;
                    else if (cell == 'b') targetB = idx;

                    idx++;
                }
            }
        }

        // Precompute successors for all free cells
        successors = new ArrayList[freeCellsCount];
        for (int i = 0; i < freeCellsCount; i++) {
            successors[i] = computeSuccessors(i);
        }
    }

    /**
     * Computes all reachable cells from a given free cell in one move
     * @param cellId Identifier of the current cell
     * @return List of reachable cell identifiers
     */
    private List<Integer> computeSuccessors(int cellId) {
        List<Integer> result = new ArrayList<>();
        int r = rowOfIndex[cellId];
        int c = colOfIndex[cellId];

        // Check all 5 possible moves: N, E, S, W, WAIT
        for (int dir = 0; dir < 5; dir++) {
            int nr = r + DR[dir];
            int nc = c + DC[dir];

            if (dir == 4) {
                // WAIT move: stay in current cell
                result.add(cellId);
            } else if (nr >= 0 && nr < rows && nc >= 0 && nc < cols) {
                // Directional move: check boundaries and walls
                if (cellType[nr][nc] != '#') {
                    result.add(indexOf[nr][nc]);
                }
            }
        }
        return result;
    }

    /**
     * Checks if a pair of moves is legal
     * @param a Current position of robot A
     * @param b Current position of robot B
     * @param nextA Next position of robot A
     * @param nextB Next position of robot B
     * @return true if the move pair is legal, false otherwise
     */
    private boolean isLegalMove(int a, int b, int nextA, int nextB) {
        // Rule 1: Robots cannot occupy the same cell after move
        if (nextA == nextB) {
            return false;
        }
        // Rule 2: Robots cannot swap positions in the same tick
        if (nextA == b && nextB == a) {
            return false;
        }
        return true;
    }

    /**
     * Determines the move direction from current to next position
     * @param current Current cell index
     * @param next Next cell index
     * @return Move index (0-4), or -1 if invalid
     */
    private int getMoveIndex(int current, int next) {
        if (current == next) return 4; // WAIT

        int curRow = rowOfIndex[current];
        int curCol = colOfIndex[current];
        int nextRow = rowOfIndex[next];
        int nextCol = colOfIndex[next];

        // Determine direction
        if (nextRow == curRow - 1 && nextCol == curCol) return 0; // North
        if (nextRow == curRow && nextCol == curCol + 1) return 1; // East
        if (nextRow == curRow + 1 && nextCol == curCol) return 2; // South
        if (nextRow == curRow && nextCol == curCol - 1) return 3; // West

        return -1; // Invalid move
    }

    /**
     * Solves the robot path planning problem using BFS
     * @param map Hospital map as array of strings
     * @param T Maximum search time
     * @return Minimum makespan if solution exists, -1 otherwise
     */
    public int solve(String[] map, int T) {
        // Check if all robot positions are valid
        if (startA == -1 || startB == -1 || targetA == -1 || targetB == -1) {
            return -1;
        }

        // reachable[t][a][b]: whether state (a,b) is reachable at time t
        boolean[][][] reachable = new boolean[T + 1][freeCellsCount][freeCellsCount];

        // Predecessor information for path reconstruction
        int[][][] prevA = new int[T + 1][freeCellsCount][freeCellsCount];
        int[][][] prevB = new int[T + 1][freeCellsCount][freeCellsCount];
        int[][][] moveA = new int[T + 1][freeCellsCount][freeCellsCount];
        int[][][] moveB = new int[T + 1][freeCellsCount][freeCellsCount];

        // Initialize predecessor arrays to -1
        for (int t = 0; t <= T; t++) {
            for (int i = 0; i < freeCellsCount; i++) {
                Arrays.fill(prevA[t][i], -1);
                Arrays.fill(prevB[t][i], -1);
                Arrays.fill(moveA[t][i], -1);
                Arrays.fill(moveB[t][i], -1);
            }
        }

        // Initial state at time 0: both robots at start positions
        reachable[0][startA][startB] = true;

        // BFS over time steps
        for (int t = 0; t < T; t++) {
            for (int a = 0; a < freeCellsCount; a++) {
                for (int b = 0; b < freeCellsCount; b++) {
                    if (reachable[t][a][b]) {
                        // Try all possible moves for both robots
                        for (int nextA : successors[a]) {
                            for (int nextB : successors[b]) {
                                if (isLegalMove(a, b, nextA, nextB)) {
                                    if (!reachable[t + 1][nextA][nextB]) {
                                        reachable[t + 1][nextA][nextB] = true;
                                        prevA[t + 1][nextA][nextB] = a;
                                        prevB[t + 1][nextA][nextB] = b;
                                        moveA[t + 1][nextA][nextB] = getMoveIndex(a, nextA);
                                        moveB[t + 1][nextA][nextB] = getMoveIndex(b, nextB);
                                    }
                                }
                            }
                        }
                    }
                }
            }
        }

        // Find the earliest time when both robots reach their targets
        for (int t = 0; t <= T; t++) {
            if (reachable[t][targetA][targetB]) {
                printSchedule(t, targetA, targetB, prevA, prevB, moveA, moveB);
                return t;
            }
        }

        return -1; // No solution found within time T
    }

    /**
     * Prints the optimal synchronized schedule by backtracking
     * @param t Makespan (time when solution was found)
     * @param targetA Robot A's target cell index
     * @param targetB Robot B's target cell index
     * @param prevA Predecessor A positions array
     * @param prevB Predecessor B positions array
     * @param moveA Robot A's moves array
     * @param moveB Robot B's moves array
     */
    private void printSchedule(int t, int targetA, int targetB,
                               int[][][] prevA, int[][][] prevB,
                               int[][][] moveA, int[][][] moveB) {
        List<String> movesA = new ArrayList<>();
        List<String> movesB = new ArrayList<>();

        int currentT = t;
        int currentA = targetA;
        int currentB = targetB;

        // Backtrack from target state to initial state
        while (currentT > 0) {
            int moveIdxA = moveA[currentT][currentA][currentB];
            int moveIdxB = moveB[currentT][currentA][currentB];

            // Add moves to the beginning of the list (reverse order)
            movesA.add(0, MOVE_NAMES[moveIdxA]);
            movesB.add(0, MOVE_NAMES[moveIdxB]);

            // Move to previous state
            int prevAVal = prevA[currentT][currentA][currentB];
            int prevBVal = prevB[currentT][currentA][currentB];

            currentA = prevAVal;
            currentB = prevBVal;
            currentT--;
        }

        // Print the schedule
        System.out.println("Optimal synchronized schedule (makespan = " + t + "):");
        for (int i = 0; i < movesA.size(); i++) {
            System.out.printf("tick %d: A = %-4s B = %s%n",
                    i + 1, movesA.get(i), movesB.get(i));
        }
    }


    /**
     * Representative map examples for testing
     */
    public static void main(String[] args) {
        // Test case 1: Single-lane corridor
        System.out.println("Test Case 1: Single-lane corridor");
        String[] map1 = {
                "#########",
                "#bA...Ba#",
                "#########"
        };
        RobotPlanner01 solver1 = new RobotPlanner01(map1);
        int result1 = solver1.solve(map1, 20);
        if (result1 == -1) {
            System.out.println("No synchronized schedule exists.");
        }
        System.out.println();

        // Test case 2: Four-way intersection
        System.out.println("Test Case 2: Four-way intersection");
        String[] map2 = {
                "#######",
                "###a###",
                "###.###",
                "#A...B#",
                "###.###",
                "###b###",
                "#######"
        };
        RobotPlanner01 solver2 = new RobotPlanner01(map2);
        int result2 = solver2.solve(map2, 20);
        if (result2 == -1) {
            System.out.println("No synchronized schedule exists.");
        }
        System.out.println();

        // Test case 3: Parking bay
        System.out.println("Test Case 3: Parking bay");
        String[] map3 = {
                "#########",
                "#bA...Ba#",
                "####.####",
                "#########"
        };
        RobotPlanner01 solver3 = new RobotPlanner01(map3);
        int result3 = solver3.solve(map3, 20);
        if (result3 == -1) {
            System.out.println("No synchronized schedule exists.");
        }
        System.out.println();

        // Test case 4: Side-room staging
        System.out.println("Test Case 4: Side-room staging");
        String[] map4 = {
                "#########",
                "#A....b.#",
                "#.###.###",
                "#..B.a..#",
                "#########"
        };
        RobotPlanner01 solver4 = new RobotPlanner01(map4);
        int result4 = solver4.solve(map4, 20);
        if (result4 == -1) {
            System.out.println("No synchronized schedule exists.");
        }
        System.out.println();

        // Test case 5: Sealed target chamber
        System.out.println("Test Case 5: Sealed target chamber");
        String[] map5 = {
                "#########",
                "#A..#.a.#",
                "#.#####.#",
                "#B...b#.#",
                "#########"
        };
        RobotPlanner01 solver5 = new RobotPlanner01(map5);
        int result5 = solver5.solve(map5, 20);
        if (result5 == -1) {
            System.out.println("No synchronized schedule exists.");
        }
    }
}

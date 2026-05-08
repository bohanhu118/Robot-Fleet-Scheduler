import java.util.*;

/**
 * This program solves the synchronized schedule problem using a state graph approach.
 * Vertices represent valid robot positions (a,b) with a != b.
 * Edges represent legal move pairs between states.
 * BFS is used to find the shortest path from start to target.
 */
public class RobotPlanner02 {

    // Map dimensions
    private int rows;
    private int cols;
    private int freeCellsCount;

    // Index mapping: index <-> coordinates
    private int[] rowOfIndex;
    private int[] colOfIndex;
    private int[][] indexOf;
    private char[][] cellType;

    // Robot positions
    private int startA, startB, targetA, targetB;

    // Single-robot successors (precomputed)
    private List<Integer>[] successors;

    // Direction constants
    private static final int[] DR = {-1, 0, 1, 0, 0};
    private static final int[] DC = {0, 1, 0, -1, 0};
    private static final String[] MOVE_NAMES = {"N", "E", "S", "W", "WAIT"};

    // Adjacency list for state graph
    private List<Edge>[] adjList;

    /**
     * Edge representing a transition between states
     */
    private static class Edge {
        int targetVertexId;
        int moveA;
        int moveB;

        Edge(int targetVertexId, int moveA, int moveB) {
            this.targetVertexId = targetVertexId;
            this.moveA = moveA;
            this.moveB = moveB;
        }
    }

    /**
     * Vertex information for BFS traversal
     */
    private static class VertexInfo {
        int prevVertexId;
        int moveA;
        int moveB;
        boolean visited;
        int distance;

        VertexInfo() {
            this.prevVertexId = -1;
            this.moveA = -1;
            this.moveB = -1;
            this.visited = false;
            this.distance = -1;
        }
    }


    /**
     * Constructor: Parses map and builds the state graph
     * @param map Hospital map as array of strings
     */
    @SuppressWarnings("unchecked")
    public RobotPlanner02(String[] map) {
        parseMap(map);
        precomputeSuccessors();
        buildStateGraph();
    }

    /**
     * Parses the input map and sets up indexing
     */
    private void parseMap(String[] map) {
        this.rows = map.length;
        this.cols = map[0].length();
        this.cellType = new char[rows][cols];

        // First pass: count free cells
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

        // Second pass: assign indices and locate robots
        int idx = 0;
        startA = startB = targetA = targetB = -1;

        for (int r = 0; r < rows; r++) {
            for (int c = 0; c < cols; c++) {
                char cell = cellType[r][c];
                if (cell != '#') {
                    rowOfIndex[idx] = r;
                    colOfIndex[idx] = c;
                    indexOf[r][c] = idx;

                    if (cell == 'A') startA = idx;
                    else if (cell == 'a') targetA = idx;
                    else if (cell == 'B') startB = idx;
                    else if (cell == 'b') targetB = idx;

                    idx++;
                }
            }
        }
    }

    /**
     * Precomputes successors for all free cells
     */
    private void precomputeSuccessors() {
        successors = new ArrayList[freeCellsCount];
        for (int i = 0; i < freeCellsCount; i++) {
            successors[i] = computeSuccessors(i);
        }
    }

    /**
     * Computes reachable cells from a given free cell
     */
    private List<Integer> computeSuccessors(int cellId) {
        List<Integer> result = new ArrayList<>();
        int r = rowOfIndex[cellId];
        int c = colOfIndex[cellId];

        for (int dir = 0; dir < 5; dir++) {
            int nr = r + DR[dir];
            int nc = c + DC[dir];

            if (dir == 4) {
                result.add(cellId);  // WAIT
            } else if (nr >= 0 && nr < rows && nc >= 0 && nc < cols) {
                if (cellType[nr][nc] != '#') {
                    result.add(indexOf[nr][nc]);
                }
            }
        }
        return result;
    }

    /**
     * Checks if a pair of moves is legal
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
     * Gets move index from current to next position
     */
    private int getMoveIndex(int current, int next) {
        if (current == next) return 4;  // WAIT

        int curRow = rowOfIndex[current];
        int curCol = colOfIndex[current];
        int nextRow = rowOfIndex[next];
        int nextCol = colOfIndex[next];

        if (nextRow == curRow - 1 && nextCol == curCol) return 0;  // N
        if (nextRow == curRow && nextCol == curCol + 1) return 1;  // E
        if (nextRow == curRow + 1 && nextCol == curCol) return 2;  // S
        if (nextRow == curRow && nextCol == curCol - 1) return 3;  // W

        return -1;
    }

    /**
     * Encodes state (a,b) into a single integer ID
     * @param a Robot A position index
     * @param b Robot B position index
     * @return State ID = a * P + b
     */
    private int encodeState(int a, int b) {
        return a * freeCellsCount + b;
    }

    /**
     * Decodes state ID back to (a,b)
     * @param stateId Encoded state ID
     * @return Array {a, b}
     */
    private int[] decodeState(int stateId) {
        int a = stateId / freeCellsCount;
        int b = stateId % freeCellsCount;
        return new int[]{a, b};
    }

    /**
     * Builds the complete state graph
     * Vertices: all (a,b) with a != b
     * Edges: legal move pairs between states
     */
    private void buildStateGraph() {
        int maxVertices = freeCellsCount * freeCellsCount;
        adjList = new ArrayList[maxVertices];
        for (int i = 0; i < maxVertices; i++) {
            adjList[i] = new ArrayList<>();
        }

        // Build graph by exploring all possible states
        for (int a = 0; a < freeCellsCount; a++) {
            for (int b = 0; b < freeCellsCount; b++) {
                if (a == b) continue;  // Invalid state

                int fromId = encodeState(a, b);

                // Generate all legal successor states
                for (int nextA : successors[a]) {
                    for (int nextB : successors[b]) {
                        if (!isLegalMove(a, b, nextA, nextB)) continue;

                        int toId = encodeState(nextA, nextB);
                        int moveAIdx = getMoveIndex(a, nextA);
                        int moveBIdx = getMoveIndex(b, nextB);

                        adjList[fromId].add(new Edge(toId, moveAIdx, moveBIdx));
                    }
                }
            }
        }
    }


    /**
     * Solves the robot path planning problem using BFS on state graph
     *
     * @param map Hospital map (for interface compatibility, already processed in constructor)
     * @return Minimum makespan if solution exists, -1 otherwise
     */
    public int solve(String[] map) {
        // Check if all robot positions are valid
        if (startA == -1 || startB == -1 || targetA == -1 || targetB == -1) {
            return -1;
        }

        int startId = encodeState(startA, startB);
        int targetId = encodeState(targetA, targetB);

        int maxVertices = freeCellsCount * freeCellsCount;
        VertexInfo[] info = new VertexInfo[maxVertices];
        for (int i = 0; i < maxVertices; i++) {
            info[i] = new VertexInfo();
        }

        Queue<Integer> queue = new LinkedList<>();
        queue.offer(startId);
        info[startId].visited = true;
        info[startId].distance = 0;

        // BFS traversal
        while (!queue.isEmpty()) {
            int currentId = queue.poll();
            int currentDist = info[currentId].distance;

            // Check if we reached the target
            if (currentId == targetId) {
                printScheduleFromInfo(targetId, info, startId);
                return currentDist;
            }

            // Explore all neighbors
            for (Edge edge : adjList[currentId]) {
                if (!info[edge.targetVertexId].visited) {
                    info[edge.targetVertexId].visited = true;
                    info[edge.targetVertexId].distance = currentDist + 1;
                    info[edge.targetVertexId].prevVertexId = currentId;
                    info[edge.targetVertexId].moveA = edge.moveA;
                    info[edge.targetVertexId].moveB = edge.moveB;
                    queue.offer(edge.targetVertexId);
                }
            }
        }

        return -1;
    }

    /**
     * Prints the schedule by backtracking from target to start
     */
    private void printScheduleFromInfo(int targetId, VertexInfo[] info, int startId) {
        List<String> movesA = new ArrayList<>();
        List<String> movesB = new ArrayList<>();

        int currentId = targetId;

        while (currentId != startId) {
            movesA.add(0, MOVE_NAMES[info[currentId].moveA]);
            movesB.add(0, MOVE_NAMES[info[currentId].moveB]);
            currentId = info[currentId].prevVertexId;
        }

        int makespan = movesA.size();
        System.out.println("Optimal synchronized schedule (makespan = " + makespan + "):");
        for (int i = 0; i < movesA.size(); i++) {
            System.out.printf("tick %d: A = %-4s B = %s%n",
                    i + 1, movesA.get(i), movesB.get(i));
        }
    }


    /**
     * Representative map examples for testing
     */
    public static void main(String[] args) {
        // Test case 1: Single-lane corridor (theoretically impossible)
        System.out.println("Test Case 1: Single-lane corridor");
        String[] map1 = {
                "#########",
                "#bA...Ba#",
                "#########"
        };
        RobotPlanner02 solver1 = new RobotPlanner02(map1);
        int result1 = solver1.solve(map1);
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
        RobotPlanner02 solver2 = new RobotPlanner02(map2);
        int result2 = solver2.solve(map2);
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
        RobotPlanner02 solver3 = new RobotPlanner02(map3);
        int result3 = solver3.solve(map3);
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
        RobotPlanner02 solver4 = new RobotPlanner02(map4);
        int result4 = solver4.solve(map4);
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
                "#B...b..#",
                "#########"
        };
        RobotPlanner02 solver5 = new RobotPlanner02(map5);
        int result5 = solver5.solve(map5);
        if (result5 == -1) {
            System.out.println("No synchronized schedule exists.");
        }
    }
}
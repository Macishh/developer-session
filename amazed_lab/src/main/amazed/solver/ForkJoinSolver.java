package amazed.solver;

import amazed.maze.Maze;

import java.util.List;
import java.util.ArrayList;
import java.util.Map;
import java.util.HashMap;
import java.util.Set;
import java.util.concurrent.ConcurrentSkipListSet;

/**
 * <code>ForkJoinSolver</code> implements a solver for
 * <code>Maze</code> objects using a fork/join multi-thread
 * depth-first search.
 * <p>
 * Instances of <code>ForkJoinSolver</code> should be run by a
 * <code>ForkJoinPool</code> object.
 */

public class ForkJoinSolver
        extends SequentialSolver {
    /**
     * Creates a solver that searches in <code>maze</code> from the
     * start node to a goal.
     *
     * @param maze the maze to be searched
     */
    public ForkJoinSolver(Maze maze) {
        super(maze);
        visited = new ConcurrentSkipListSet<>();
    }

    /**
     * Creates a solver that searches in <code>maze</code> from the
     * start node to a goal, forking after a given number of visited
     * nodes.
     *
     * @param maze      the maze to be searched
     * @param forkAfter the number of steps (visited nodes) after
     *                  which a parallel task is forked; if
     *                  <code>forkAfter &lt;= 0</code> the solver never
     *                  forks new tasks
     */
    public ForkJoinSolver(Maze maze, int forkAfter) {
        this(maze);
        this.forkAfter = forkAfter;
    }

    /**
     * Searches for and returns the path, as a list of node
     * identifiers, that goes from the start node to a goal node in
     * the maze. If such a path cannot be found (because there are no
     * goals, or all goals are unreacheable), the method returns
     * <code>null</code>.
     *
     * @return the list of node identifiers from the start node to a
     *         goal node in the maze; <code>null</code> if such a path cannot
     *         be found.
     */
    @Override
    public List<Integer> compute() {
        return parallelSearch();
    }

    private List<Integer> parallelSearch() {

        // List to keep track of all children a parent have to look through if any of
        // them have found the goal
        List<ForkJoinSolver> children = new ArrayList<>();

        // Create a new player with position at start, i.e. (0, 0) for the first player
        // Other players will have frontier.pop() as the start position
        int player = maze.newPlayer(start);
        // Int to keep track of the iterations and when a new thread should be spawned
        int iterations = 0;
        // Adds the first node to the frontier
        frontier.push(start);

        // Loop while frontier is not empty
        while (!frontier.empty()) {
            // If iterations mod forkAfter != 0, i.e not time to spawn a new thread, we
            // check if goal is reached or go to the next node
            if (iterations % forkAfter != 0 || frontier.size() <= 1) {
                int node = frontier.pop();
                // if goal is reached return path from start to node
                if (maze.hasGoal(node)) {
                    maze.move(player, node);
                    return pathFromTo(start, node);
                }
                // If node is not visited, add it to visited and add its neighbours to the
                // frontier (if the neigbours are not already visited)
                // Also add the path from the node to the neighbour in the predecessor
                if (!visited.contains(node)) {
                    visited.add(node);
                    maze.move(player, node);
                    for (int neighbor : maze.neighbors(node)) {
                        if (!visited.contains(neighbor)) {
                            predecessor.put(neighbor, node);
                            frontier.push(neighbor);
                        }
                    }
                }
                iterations++;
            } else {
                // If iterations mod forkAfter == 0, and frontier is larger than 1 we spawn a
                // new thead
                ForkJoinSolver newPlayer = new ForkJoinSolver(maze, forkAfter);
                // Let the new player start at next position in the frontier
                newPlayer.start = frontier.pop();
                // Use the same visited list as the parent / other threads
                newPlayer.visited = visited;
                children.add(newPlayer);
                newPlayer.fork();
            }
        }

        // If goal has not been found, we check if any of the children have found the
        // goal
        for (ForkJoinSolver child : children) {
            // If the child has returned anything we check the path, otherwise wait
            // until child is done
            List<Integer> previousPath = child.join();
            // Check if path is found, otherwise continue
            if (previousPath != null) {
                // Combine the path from start to the childs start
                List<Integer> combinedPath = pathFromTo(start, previousPath.remove(0));
                // Add the path from the childs start to the goal
                combinedPath.addAll(previousPath);
                // Return the combined path for both threads
                return combinedPath;
            }
        }

        // Return null if no path was found
        return null;
    }
}

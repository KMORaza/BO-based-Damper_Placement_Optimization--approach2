package solution.damper_placement.source;

import java.util.ArrayList;
import java.util.Comparator;
import java.util.List;
import java.util.stream.Collectors;

public class ParetoFront {
    private final int maxSize;
    private final List<Solution> solutions;

    public static class Solution {
        public final double[] params;
        public final String profile;
        public final double[] objectives;
        public int dominationCount;
        public List<Solution> dominatedSolutions;
        public double crowdingDistance;

        public Solution(double[] params, String profile, double[] objectives) {
            this.params = params.clone();
            this.profile = profile;
            this.objectives = objectives.clone();
            this.dominationCount = 0;
            this.dominatedSolutions = new ArrayList<>();
            this.crowdingDistance = 0.0;
        }
    }

    public ParetoFront(int maxSize) {
        this.maxSize = maxSize;
        this.solutions = new ArrayList<>();
    }

    public void addSolution(double[] params, String profile, double[] objectives) {
        Solution newSolution = new Solution(params, profile, objectives);
        List<List<Solution>> fronts = nonDominatedSort(solutions, newSolution);
        solutions.clear();
        int remaining = maxSize;

        for (List<Solution> front : fronts) {
            if (front.isEmpty()) continue;
            computeCrowdingDistance(front);
            front.sort(Comparator.comparingDouble(s -> -s.crowdingDistance));
            for (Solution s : front) {
                if (solutions.size() < remaining) {
                    solutions.add(s);
                } else {
                    break;
                }
            }
            if (solutions.size() >= remaining) break;
        }
    }

    public List<Solution> getSolutions() {
        return new ArrayList<>(solutions);
    }

    private List<List<Solution>> nonDominatedSort(List<Solution> current, Solution newSolution) {
        List<Solution> all = new ArrayList<>(current);
        all.add(newSolution);
        List<List<Solution>> fronts = new ArrayList<>();
        fronts.add(new ArrayList<>());

        for (Solution s : all) {
            s.dominationCount = 0;
            s.dominatedSolutions.clear();
            for (Solution other : all) {
                if (s == other) continue;
                if (dominates(s, other)) {
                    s.dominatedSolutions.add(other);
                } else if (dominates(other, s)) {
                    s.dominationCount++;
                }
            }
            if (s.dominationCount == 0) {
                fronts.get(0).add(s);
            }
        }

        int i = 0;
        while (!fronts.get(i).isEmpty()) {
            List<Solution> nextFront = new ArrayList<>();
            for (Solution s : fronts.get(i)) {
                for (Solution dominated : s.dominatedSolutions) {
                    dominated.dominationCount--;
                    if (dominated.dominationCount == 0) {
                        nextFront.add(dominated);
                    }
                }
            }
            fronts.add(nextFront);
            i++;
        }
        return fronts;
    }

    private boolean dominates(Solution s1, Solution s2) {
        boolean betterInAny = false;
        for (int i = 0; i < s1.objectives.length; i++) {
            if (s1.objectives[i] > s2.objectives[i]) {
                return false;
            }
            if (s1.objectives[i] < s2.objectives[i]) {
                betterInAny = true;
            }
        }
        return betterInAny;
    }

    private void computeCrowdingDistance(List<Solution> front) {
        int n = front.size();
        int m = front.get(0).objectives.length;
        for (Solution s : front) {
            s.crowdingDistance = 0.0;
        }
        for (int i = 0; i < m; i++) {
            final int objIdx = i;
            front.sort(Comparator.comparingDouble(s -> s.objectives[objIdx]));
            front.get(0).crowdingDistance = Double.POSITIVE_INFINITY;
            front.get(n - 1).crowdingDistance = Double.POSITIVE_INFINITY;
            double maxObj = front.get(n - 1).objectives[i];
            double minObj = front.get(0).objectives[i];
            double range = maxObj - minObj;
            if (range < 1e-6) continue;
            for (int j = 1; j < n - 1; j++) {
                front.get(j).crowdingDistance += (front.get(j + 1).objectives[i] - front.get(j - 1).objectives[i]) / range;
            }
        }
    }
}
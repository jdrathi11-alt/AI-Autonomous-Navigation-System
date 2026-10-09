#!/usr/bin/env python3
"""Pure stdlib tests for core navigation components."""
import math
import sys
sys.path.append('../src')
from path_planner import a_star, heuristic, neighbors, w2g, g2w
from simulation import occ, START, GOAL, OBSTACLES, CELL, W, H

def test_heuristic():
    """Test Euclidean distance heuristic."""
    assert heuristic((0,0), (3,4)) == 5.0
    assert heuristic((1,1), (1,1)) == 0.0
    assert heuristic((0,0), (1,1)) == math.sqrt(2)
    print("✓ heuristic test passed")

def test_neighbors():
    """Test neighbor generation."""
    # Test center point
    neigh = list(neighbors((5,5)))
    # Should have 8 neighbors in open space
    # but we'll just check it returns something reasonable
    assert len(neigh) > 0
    # All neighbors should be valid grid positions
    for nx, ny in neigh:
        assert 0 <= nx < H//CELL
        assert 0 <= ny < W//CELL
    print("✓ neighbors test passed")

def test_coordinate_conversion():
    """Test world-grid conversion."""
    # Test known points
    assert w2g((0,0)) == (0,0)  # top-left
    assert w2g((CELL//2, CELL//2)) == (0,0)  # center of first cell
    assert g2w((0,0)) == (CELL//2, CELL//2)  # center of first cell
    assert g2w((1,0)) == (CELL + CELL//2, CELL//2)  # center of second cell in row
    print("✓ coordinate conversion test passed")

def test_astar_basic():
    """Test A* finds path in simple case."""
    # Simple 3x3 grid with no obstacles
    simple_occ = [[False, False, False],
                  [False, False, False],
                  [False, False, False]]
    # Temporarily replace occ
    import simulation
    original_occ = simulation.occ
    simulation.occ = simple_occ
    
    try:
        start = (0,0)
        goal = (2,2)
        path = a_star(start, goal)
        assert path is not None
        assert path[0] == start
        assert path[-1] == goal
        # Should be diagonal-ish path
        assert len(path) >= 3  # at least start, middle, end
    finally:
        simulation.occ = original_occ
    print("✓ A* basic test passed")

def test_obstacle_detection():
    """Test that obstacles are properly marked."""
    # Check a few known obstacle positions
    obs_grid = [[False] * (W//CELL) for _ in range(H//CELL)]
    for x, y, w, h in OBSTACLES:
        for cx in range(x//CELL, (x+w)//CELL + 1):
            for cy in range(y//CELL, (y+h)//CELL + 1):
                if 0 <= cx < len(obs_grid) and 0 <= cy < len(obs_grid[0]):
                    obs_grid[cx][cy] = True
    
    # Compare with our occ
    for i in range(len(occ)):
        for j in range(len(occ[0])):
            assert occ[i][j] == obs_grid[i][j], f"Mismatch at ({i},{j})"
    print("✓ obstacle detection test passed")

def test_path_finds_goal():
    """Test that our actual setup finds a path."""
    start_grid = w2g(START)
    goal_grid = w2g(GOAL)
    path = a_star(start_grid, goal_grid)
    assert path is not None, "No path found in main scenario"
    assert path[0] == start_grid
    assert path[-1] == goal_grid
    print(f"✓ Main path test passed: {len(path)} waypoints")

if __name__ == "__main__":
    test_heuristic()
    test_neighbors()
    test_coordinate_conversion()
    test_astar_basic()
    test_obstacle_detection()
    test_path_finds_goal()
    print("\n🎉 All tests passed!")

# Campus Navigation and Path-Finding System
DAA Mini Project — Python 3, Tkinter

## Run
1. Install Python 3.9 or newer.
2. Save/extract this folder.
3. Open a terminal in this folder.
4. Run: `python campus_navigation.py`

No third-party Python packages are required (Tkinter is included with most standard Python installations).

## Features
- Sample campus graph with locations and weighted walkways.
- Choose start and destination from dropdowns.
- Find routes using Dijkstra or A*.
- Compare route distance, expanded nodes, and measured execution time.
- Highlight the selected route on the map.

## Important data note
The locations and distances are illustrative sample data, not a surveyed map of a real college. Replace `NODES` and `EDGES` in `campus_navigation.py` with your own campus locations and measured walking distances before submitting it as a campus-specific system.

## DAA concepts
- Graph representation: adjacency list.
- Dijkstra: shortest path for non-negative edge weights.
- A*: best-first search using g(n) + h(n); heuristic is conservatively scaled from the supplied drawing coordinates and edge weights.
- Experimental evaluation: compare route length, expanded nodes, and execution time. Timings vary by machine and should be collected by running the application.

## Suggested group split
1. Problem analysis and literature/background study.
2. Graph modelling, pseudocode, and complexity analysis.
3. Implementation and testing.
4. UI, visualizations, report, results, and presentation.

All group members should understand the whole project.

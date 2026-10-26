"""
Campus Navigation and Path-Finding System
DAA Mini Project | Python 3 | Tkinter GUI
Algorithms: Dijkstra's Algorithm and A* Search
Run: python campus_navigation.py
No third-party packages are required.
"""
import tkinter as tk
from tkinter import ttk, messagebox
import heapq
import math
import time

# Sample campus graph. Edge weights represent illustrative walking distances in metres.
# Replace these locations/distances with surveyed data from your own campus.
NODES = {
    "Main Gate": (70, 260),
    "Admin Block": (210, 160),
    "Academic Block": (360, 130),
    "Central Library": (520, 190),
    "Computer Lab": (390, 300),
    "Canteen": (250, 360),
    "Sports Ground": (590, 370),
    "Hostel": (720, 250),
    "Auditorium": (115, 420),
    "Parking": (70, 120),
}
EDGES = [
    ("Main Gate", "Admin Block", 120),
    ("Main Gate", "Canteen", 155),
    ("Main Gate", "Auditorium", 175),
    ("Main Gate", "Parking", 105),
    ("Admin Block", "Academic Block", 135),
    ("Admin Block", "Canteen", 140),
    ("Admin Block", "Parking", 150),
    ("Academic Block", "Central Library", 170),
    ("Academic Block", "Computer Lab", 180),
    ("Canteen", "Computer Lab", 145),
    ("Canteen", "Auditorium", 115),
    ("Computer Lab", "Central Library", 155),
    ("Computer Lab", "Sports Ground", 210),
    ("Central Library", "Sports Ground", 185),
    ("Central Library", "Hostel", 230),
    ("Sports Ground", "Hostel", 175),
    ("Admin Block", "Auditorium", 190),
]
GRAPH = {name: [] for name in NODES}
for a, b, distance in EDGES:
    GRAPH[a].append((b, distance))
    GRAPH[b].append((a, distance))


def dijkstra(graph, start, goal):
    """Return (path, total_distance, expanded_nodes)."""
    queue = [(0, start)]
    distances = {node: math.inf for node in graph}
    previous = {}
    distances[start] = 0
    visited = set()
    expanded = 0
    while queue:
        current_distance, current = heapq.heappop(queue)
        if current in visited:
            continue
        visited.add(current)
        expanded += 1
        if current == goal:
            break
        for neighbor, weight in graph[current]:
            candidate = current_distance + weight
            if candidate < distances[neighbor]:
                distances[neighbor] = candidate
                previous[neighbor] = current
                heapq.heappush(queue, (candidate, neighbor))
    if distances[goal] == math.inf:
        return [], math.inf, expanded
    path = []
    node = goal
    while node != start:
        path.append(node)
        node = previous[node]
    path.append(start)
    path.reverse()
    return path, distances[goal], expanded


def heuristic_scale(graph, coords):
    """Find a conservative Euclidean heuristic scale for the given weighted graph."""
    ratios = []
    for a, b, weight in EDGES:
        x1, y1 = coords[a]
        x2, y2 = coords[b]
        geometric = math.hypot(x2 - x1, y2 - y1)
        if geometric > 0:
            ratios.append(weight / geometric)
    return min(ratios) if ratios else 0.0


def astar(graph, coords, start, goal):
    """A* with an admissible Euclidean heuristic; returns (path, distance, expanded)."""
    scale = heuristic_scale(graph, coords)
    def h(node):
        x1, y1 = coords[node]
        x2, y2 = coords[goal]
        return math.hypot(x2 - x1, y2 - y1) * scale

    queue = [(h(start), 0, start)]
    g_score = {node: math.inf for node in graph}
    previous = {}
    g_score[start] = 0
    closed = set()
    expanded = 0
    while queue:
        _, current_g, current = heapq.heappop(queue)
        if current in closed:
            continue
        if current == goal:
            expanded += 1
            break
        closed.add(current)
        expanded += 1
        for neighbor, weight in graph[current]:
            tentative = current_g + weight
            if tentative < g_score[neighbor]:
                g_score[neighbor] = tentative
                previous[neighbor] = current
                heapq.heappush(queue, (tentative + h(neighbor), tentative, neighbor))
    if g_score[goal] == math.inf:
        return [], math.inf, expanded
    path = []
    node = goal
    while node != start:
        path.append(node)
        node = previous[node]
    path.append(start)
    path.reverse()
    return path, g_score[goal], expanded


class CampusApp:
    def __init__(self, root):
        self.root = root
        self.root.title("Campus Navigation and Path-Finding System")
        self.root.geometry("1060x720")
        self.root.minsize(900, 620)
        self.start_var = tk.StringVar(value="Main Gate")
        self.goal_var = tk.StringVar(value="Central Library")
        self.algorithm_var = tk.StringVar(value="Compare both")
        self.status_var = tk.StringVar(value="Choose a start and destination, then Find Route.")
        self.results = {}
        self._build_ui()
        self.draw_map()

    def _build_ui(self):
        header = ttk.Frame(self.root, padding=(14, 12))
        header.pack(fill="x")
        ttk.Label(header, text="Campus Navigation & Path-Finding System",
                  font=("Segoe UI", 17, "bold")).pack(anchor="w")
        ttk.Label(header, text="DAA Mini Project • Shortest paths with Dijkstra and A*",
                  font=("Segoe UI", 10)).pack(anchor="w", pady=(3, 0))

        controls = ttk.LabelFrame(self.root, text="Route settings", padding=10)
        controls.pack(fill="x", padx=14, pady=(0, 10))
        names = list(NODES.keys())
        ttk.Label(controls, text="Start location").grid(row=0, column=0, sticky="w", padx=(0, 6))
        ttk.Combobox(controls, textvariable=self.start_var, values=names, state="readonly", width=20)\
            .grid(row=0, column=1, padx=(0, 16), pady=3)
        ttk.Label(controls, text="Destination").grid(row=0, column=2, sticky="w", padx=(0, 6))
        ttk.Combobox(controls, textvariable=self.goal_var, values=names, state="readonly", width=20)\
            .grid(row=0, column=3, padx=(0, 16), pady=3)
        ttk.Label(controls, text="Algorithm").grid(row=0, column=4, sticky="w", padx=(0, 6))
        ttk.Combobox(controls, textvariable=self.algorithm_var,
                     values=["Compare both", "Dijkstra", "A* Search"],
                     state="readonly", width=16).grid(row=0, column=5, padx=(0, 10), pady=3)
        ttk.Button(controls, text="Find Route", command=self.find_route).grid(row=0, column=6, padx=4)
        ttk.Button(controls, text="Reset", command=self.reset).grid(row=0, column=7, padx=4)

        body = ttk.Frame(self.root, padding=(14, 0, 14, 8))
        body.pack(fill="both", expand=True)
        map_frame = ttk.LabelFrame(body, text="Illustrative campus map (nodes = locations; lines = walkways)", padding=5)
        map_frame.pack(side="left", fill="both", expand=True, padx=(0, 10))
        self.canvas = tk.Canvas(map_frame, bg="#f7f9fc", highlightthickness=0, width=720, height=500)
        self.canvas.pack(fill="both", expand=True)

        right = ttk.Frame(body, width=275)
        right.pack(side="right", fill="y")
        right.pack_propagate(False)
        results_frame = ttk.LabelFrame(right, text="Route results", padding=10)
        results_frame.pack(fill="x")
        self.result_text = tk.Text(results_frame, height=18, width=31, wrap="word",
                                   font=("Segoe UI", 9), relief="flat", background="#fbfbfb")
        self.result_text.pack(fill="both", expand=True)
        self.result_text.insert("1.0", "Route details will appear here.\n\nDistances are sample values and should be replaced with actual campus measurements.")
        self.result_text.configure(state="disabled")
        legend = ttk.LabelFrame(right, text="Legend", padding=10)
        legend.pack(fill="x", pady=10)
        ttk.Label(legend, text="● Location / building", foreground="#34495e").pack(anchor="w")
        ttk.Label(legend, text="━ Selected shortest route", foreground="#e4572e").pack(anchor="w")
        ttk.Label(legend, text="━ Other walkway", foreground="#aab4c3").pack(anchor="w")

        footer = ttk.Label(self.root, textvariable=self.status_var, relief="sunken", anchor="w", padding=6)
        footer.pack(fill="x", side="bottom")

    def draw_map(self, paths=None):
        self.canvas.delete("all")
        paths = paths or []
        width = max(self.canvas.winfo_width(), 720)
        height = max(self.canvas.winfo_height(), 500)
        # Scale the reference layout to the current canvas.
        xs = [p[0] for p in NODES.values()]
        ys = [p[1] for p in NODES.values()]
        min_x, max_x = min(xs), max(xs)
        min_y, max_y = min(ys), max(ys)
        margin = 45
        def xy(name):
            x, y = NODES[name]
            px = margin + (x - min_x) / max(1, max_x - min_x) * (width - 2 * margin)
            py = margin + (y - min_y) / max(1, max_y - min_y) * (height - 2 * margin)
            return px, py

        selected_edges = set()
        for path in paths:
            for a, b in zip(path, path[1:]):
                selected_edges.add(frozenset((a, b)))
        for a, b, distance in EDGES:
            x1, y1 = xy(a)
            x2, y2 = xy(b)
            chosen = frozenset((a, b)) in selected_edges
            color = "#e4572e" if chosen else "#aab4c3"
            thickness = 4 if chosen else 2
            self.canvas.create_line(x1, y1, x2, y2, fill=color, width=thickness)
            mx, my = (x1+x2)/2, (y1+y2)/2
            self.canvas.create_rectangle(mx-16, my-9, mx+16, my+9, fill="#ffffff", outline="")
            self.canvas.create_text(mx, my, text=str(distance), fill="#455468", font=("Segoe UI", 8))
        for name, (x, y) in NODES.items():
            px, py = xy(name)
            self.canvas.create_oval(px-8, py-8, px+8, py+8, fill="#2878b5", outline="white", width=2)
            self.canvas.create_text(px, py+17, text=name, fill="#263445", font=("Segoe UI", 9, "bold"))

    def find_route(self):
        start, goal = self.start_var.get(), self.goal_var.get()
        if start == goal:
            messagebox.showinfo("Same location", "Choose different start and destination locations.")
            return
        mode = self.algorithm_var.get()
        self.results = {}
        if mode in ("Compare both", "Dijkstra"):
            t0 = time.perf_counter()
            result = dijkstra(GRAPH, start, goal)
            elapsed = (time.perf_counter() - t0) * 1000
            self.results["Dijkstra"] = (*result, elapsed)
        if mode in ("Compare both", "A* Search"):
            t0 = time.perf_counter()
            result = astar(GRAPH, NODES, start, goal)
            elapsed = (time.perf_counter() - t0) * 1000
            self.results["A* Search"] = (*result, elapsed)

        paths = [item[0] for item in self.results.values() if item[0]]
        self.draw_map(paths)
        lines = [f"Start: {start}", f"Destination: {goal}", ""]
        for name, (path, distance, expanded, elapsed) in self.results.items():
            lines += [name, "-" * len(name)]
            if path:
                lines.append("Route: " + " → ".join(path))
                lines.append(f"Total distance: {distance:.0f} m")
            else:
                lines.append("No route found.")
            lines.append(f"Nodes expanded: {expanded}")
            lines.append(f"Execution time: {elapsed:.4f} ms")
            lines.append("")
        if len(self.results) == 2:
            d = self.results["Dijkstra"]
            a = self.results["A* Search"]
            lines.append("Comparison")
            lines.append("Both algorithms should return the same minimum distance for this graph.")
            lines.append(f"Distance match: {'Yes' if d[1] == a[1] else 'Check results'}")
            lines.append("Execution times can vary by computer and run.")
        self.result_text.configure(state="normal")
        self.result_text.delete("1.0", "end")
        self.result_text.insert("1.0", "\n".join(lines))
        self.result_text.configure(state="disabled")
        self.status_var.set("Route calculated. Sample distances only; replace with measured campus data.")

    def reset(self):
        self.start_var.set("Main Gate")
        self.goal_var.set("Central Library")
        self.algorithm_var.set("Compare both")
        self.results = {}
        self.draw_map()
        self.result_text.configure(state="normal")
        self.result_text.delete("1.0", "end")
        self.result_text.insert("1.0", "Route details will appear here.\n\nDistances are sample values and should be replaced with actual campus measurements.")
        self.result_text.configure(state="disabled")
        self.status_var.set("Choose a start and destination, then Find Route.")


if __name__ == "__main__":
    root = tk.Tk()
    app = CampusApp(root)
    root.mainloop()

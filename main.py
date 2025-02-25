import heapq
import time
from collections import defaultdict, deque
from transmilenio_map import create_transmilenio_map, get_station_list

# Función BFS para calcular la distancia mínima en saltos desde cada nodo al objetivo
def bfs_min_distance(graph, target):
    """
    Calcula la distancia mínima en saltos desde cada nodo al objetivo usando BFS.
    
    Args:
        graph (dict): Grafo del sistema de transporte.
        target (int): Índice del nodo objetivo.
    
    Returns:
        dict: Diccionario con las distancias mínimas en saltos desde cada nodo al objetivo.
    """
    dist = {target: 0}  # Distancia inicial al objetivo es 0
    queue = deque([target])
    while queue:
        current = queue.popleft()
        for edge in graph[current]:
            neighbor = edge['target']
            if neighbor not in dist:
                dist[neighbor] = dist[current] + 1  # Aumentamos en 1 salto
                queue.append(neighbor)
    return dist

# Clase base ApexSearch
class ApexSearch:
    def __init__(self, graph, source, target, eps):
        """
        Clase base para la búsqueda Apex.
        
        Args:
            graph (dict): Grafo del sistema.
            source (int): Nodo origen.
            target (int): Nodo objetivo.
            eps (float): Factor de relajación.
        """
        self.graph = graph
        self.source = source
        self.target = target
        self.eps = eps

# Implementación de ApexSearchContinue
class ApexSearchContinue(ApexSearch):
    def __init__(self, graph, source, target, eps, signs, heuristics_min, heuristics_max):
        """
        Implementación de ApexSearch para búsqueda continua.
        
        Args:
            signs (list): Lista de signos para los objetivos (+1 para minimizar, -1 para maximizar).
            heuristics_min (list): Heurísticas para minimización.
            heuristics_max (list): Heurísticas para maximización.
        """
        super().__init__(graph, source, target, eps)
        self.signs = signs
        self.heuristics_min = heuristics_min
        self.heuristics_max = heuristics_max
        self.pruned_list = []
        self.solutions = []  # Lista de (costo, ruta)
        self.num_expansion = 0
        self.num_generation = 0
        self.restart_from_scratch()

    def restart_from_scratch(self):
        """Reinicia la búsqueda desde cero."""
        self.pruned_list = [(self.source, [0, 0, 0])]
        self.solutions = []
        self.num_expansion = 0
        self.num_generation = 0

    def get_transformed_cost(self, edge, total_time, total_passengers):
        """
        Transforma el costo de una arista considerando los signos de los objetivos.
        
        Returns:
            list: Costo transformado [tiempo, distancia, -pasajeros/min].
        """
        transformed = [0] * 3
        transformed[0] = self.signs[0] * (edge['cost'][0] * (1 + edge['congestion']))  # Tiempo
        transformed[1] = self.signs[1] * edge['cost'][1]  # Distancia
        avg_passengers = total_passengers / total_time if total_time > 0 else 0
        transformed[2] = self.signs[2] * avg_passengers  # -Pasajeros/min
        return transformed

    def get_transformed_heuristic(self, state):
        """
        Obtiene la heurística transformada para un nodo.
        
        Returns:
            list: Heurística transformada.
        """
        h_min = self.heuristics_min[state]
        h_max = self.heuristics_max[state]
        return [
            self.signs[0] * h_min[0],
            self.signs[1] * h_min[1],
            self.signs[2] * h_max[2]
        ]

    def solve(self, time_limit):
        """
        Resuelve el problema de búsqueda dentro de un límite de tiempo.
        
        Args:
            time_limit (float): Tiempo máximo de ejecución en segundos.
        """
        open_list = []
        g_values = defaultdict(lambda: [0, 0, 0])
        paths = {self.source: [self.source]}  # Rastrear caminos
        start_time = time.time()

        h = self.get_transformed_heuristic(self.source)
        heapq.heappush(open_list, (tuple([-x for x in h]), self.source))  # Max-heap emulation
        g_values[self.source] = [0, 0, 0]
        visited = set()

        while open_list and (time.time() - start_time) <= time_limit:
            f_val, current = heapq.heappop(open_list)
            f_val = [-x for x in f_val]  # Restore from negated values
            if current in visited:
                continue
            visited.add(current)
            self.num_generation += 1

            print(f"Exploring node {current}, f_val: {f_val}")

            if current == self.target:
                original_cost = self.get_original_cost(g_values[current])
                path = paths[current]
                self.solutions.append((original_cost, path))
                print(f"Solution found: {original_cost}, Path: {path}")

            self.num_expansion += 1
            for edge in self.graph[current]:
                total_time = g_values[current][0] + edge['cost'][0] * (1 + edge['congestion'])
                total_passengers = -g_values[current][2] + edge['cost'][2]
                next_g = self.get_transformed_cost(edge, total_time, total_passengers)
                for i in range(3):
                    next_g[i] += g_values[current][i]

                target = edge['target']
                if target not in g_values or self.is_better(next_g, g_values[target]):
                    g_values[target] = next_g
                    paths[target] = paths[current] + [target]  # Actualizar ruta
                    next_f = [next_g[i] + self.get_transformed_heuristic(target)[i] for i in range(3)]
                    heapq.heappush(open_list, (tuple([-x for x in next_f]), target))
                    print(f"Added to open: {target}, f_val: {next_f}")
                else:
                    self.pruned_list.append((target, next_g))

        print(f"Nodes expanded: {self.num_expansion}, Solutions: {len(self.solutions)}")

    def is_better(self, a, b):
        """
        Compara dos costos para determinar si 'a' es mejor que 'b'.
        
        Returns:
            bool: True si 'a' es mejor, False en caso contrario.
        """
        better = False
        for i in range(3):
            if a[i] < b[i]:
                better = True
            if a[i] > b[i]:
                return False
        return better

    def get_original_cost(self, transformed):
        """Convierte el costo transformado al original."""
        return [transformed[0], transformed[1], -transformed[2]]

    def get_solutions(self):
        """Devuelve las soluciones encontradas."""
        return self.solutions

    def get_num_expansion(self):
        """Devuelve el número de nodos expandidos."""
        return self.num_expansion

# Implementación de AnytimeApex
class AnytimeApex:
    def __init__(self, graph, source, target, eps, signs, heuristics_min, heuristics_max):
        """
        Implementación de Anytime Apex para búsqueda multiobjetivo.
        
        Args:
            graph (dict): Grafo del sistema.
            source (int): Nodo origen.
            target (int): Nodo objetivo.
            eps (float): Factor de relajación inicial.
            signs (list): Signos de los objetivos.
            heuristics_min (list): Heurísticas para minimización.
            heuristics_max (list): Heurísticas para maximización.
        """
        self.graph = graph
        self.search = ApexSearchContinue(graph, source, target, eps, signs, heuristics_min, heuristics_max)
        self.solution_log = []  # Lista de (costo, ruta)
        self.start_time = time.time()
        self.decrease_factor = 4.0

    def solve(self, time_limit):
        """
        Resuelve el problema ajustando eps dinámicamente.
        
        Args:
            time_limit (float): Tiempo máximo de ejecución en segundos.
        """
        eps = 0.2
        restart = True

        while (time.time() - self.start_time) <= time_limit:
            print(f"Running with eps: {eps}")
            self.search.eps = eps
            self.search.solve(time_limit - (time.time() - self.start_time))
            new_solutions = self.search.get_solutions()
            # Filtrar duplicados basados en costos
            for sol_cost, sol_path in new_solutions:
                if not any(cost == sol_cost for cost, _ in self.solution_log):
                    self.solution_log.append((sol_cost, sol_path))
            print(f"Current unique solutions: {[(cost, path) for cost, path in self.solution_log]}")
            if restart:
                self.search.restart_from_scratch()
            eps /= self.decrease_factor
            if eps < 0.001:
                break

    def get_solution_log(self):
        """Devuelve el registro de soluciones."""
        return self.solution_log

def main():
    # Crear el mapa de TransMilenio
    graph, station_to_idx = create_transmilenio_map()
    stations = get_station_list()

    # Agregar un camino alternativo para tener múltiples soluciones
    # Desde Calle 127 (índice 8) a Biblioteca (índice 44) directamente
    graph[8].append({'target': 44, 'cost': [50, 20, 200], 'congestion': 0.05})  # Ruta más rápida, más pasajeros

    # Definir estaciones: Portal Norte (0) a Portal 20 de Julio (45)
    source = station_to_idx["Portal Norte"]
    target = station_to_idx["Portal 20 de Julio"]
    start_station = "Portal Norte"
    end_station = "Portal 20 de Julio"

    # Definir signos: [minimizar tiempo, minimizar distancia, maximizar pasajeros]
    signs = [1, 1, -1]

    # Calcular distancia mínima en saltos desde cada nodo al target
    dist_min = bfs_min_distance(graph, target)

    # Heurísticas mejoradas
    heuristics_min = [[dist_min.get(i, 0) * 5, dist_min.get(i, 0) * 1, 0] if i in dist_min else [0, 0, 0] for i in range(len(stations))]
    heuristics_max = [[0, 0, 200] for _ in range(len(stations))]

    print(f"Buscando rutas desde {start_station} (índice {source}) hasta {end_station} (índice {target})")

    # Resolver con AnytimeApex
    solver = AnytimeApex(graph, source, target, 0.2, signs, heuristics_min, heuristics_max)
    solver.solve(10)  # Límite de 10 segundos

    # Imprimir resultados
    solutions = solver.get_solution_log()
    if not solutions:
        print(f"No se encontraron soluciones desde {start_station} hasta {end_station}.")
    else:
        print(f"\nSoluciones encontradas en el frente de Pareto aproximado desde {start_station} hasta {end_station}:")
        for i, (cost, path) in enumerate(solutions, 1):
            path_names = [stations[idx] for idx in path]
            print(f"Solución {i}:")
            print(f"  Tiempo total: {cost[0]} min, Distancia total: {cost[1]} km, Promedio de pasajeros: {cost[2]} pasajeros/min")
            print(f"  Ruta: {' -> '.join(path_names)}")

if __name__ == "__main__":
    main()
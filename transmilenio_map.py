from collections import defaultdict

# Number of objectives
N = 3

# AdjacencyMatrix class
class AdjacencyMatrix:
    def __init__(self):
        self.adj_list = defaultdict(list)
        self.num_nodes = 0
        self.num_objectives = N

    def add_edge(self, from_node, to_node, cost, congestion):
        self.adj_list[from_node].append({'target': to_node, 'cost': cost, 'congestion': congestion})

    def __getitem__(self, idx):
        return self.adj_list[idx]

    def __len__(self):
        return self.num_nodes

    def get_num_of_objectives(self):
        return self.num_objectives

# Function to create the TransMilenio map
def create_transmilenio_map():
    graph = AdjacencyMatrix()
    graph.num_nodes = 147  # Total stations

    # Mapping of stations to indices
    station_to_idx = {
        # Troncal NQS (Norte-Quito-Sur)
        "Portal Norte": 0, "Toberin": 1, "Calle 161": 2, "Mazuélen": 3, "Calle 146": 4,
        "Calle 142": 5, "Alcalá": 6, "Prado": 7, "Calle 127": 8, "Pepe Sierra": 9,
        "Calle 106": 10, "Calle 100": 11, "Virrey": 12, "Calle 85": 13, "Héroes": 14,
        "Calle 76": 15, "Calle 72": 16, "Flores": 17, "Calle 63": 18, "Calle 57": 19,
        "Marly": 20, "Calle 45": 21, "Avenida 39": 22, "Calle 34": 23, "Calle 26": 24,
        "Calle 22": 25, "Calle 19": 26, "Avenida Jiménez": 27, "Comuneros": 28,
        "Hospitales": 29, "Ricaurte": 30, "CDS Carrera 32": 31, "Avenida NQS Calle 30 Sur": 32,
        "Santa Isabel": 33, "Sevillana": 34, "NQS Calle 38A Sur": 35, "NQS Calle 40 Sur": 36,
        "Portal Sur": 37, "Bosa": 38, "La Despensa": 39, "León XIII": 40, "Terreros": 41,
        "Portal Tunal": 42, "Parque": 43, "Biblioteca": 44, "Portal 20 de Julio": 45,
        
        # Troncal Caracas
        "Portal Usme": 46, "Molinos": 47, "Consuelo": 48, "Santa Lucía": 49, "Calle 40 Sur": 50,
        "Quiroga": 51, "Olaya": 52, "Restrepo": 53, "Fucha": 54, "Tercer Milenio": 55,
        "Avenida Jiménez (Caracas)": 56, "De La Sabana": 57, "Calle 26 (Caracas)": 58,
        "Calle 34 (Caracas)": 59, "Calle 45 (Caracas)": 60, "Marly (Caracas)": 61,
        "Calle 57 (Caracas)": 62, "Calle 63 (Caracas)": 63, "Flores (Caracas)": 64,
        "Calle 72 (Caracas)": 65, "Calle 76 (Caracas)": 66, "Héroes (Caracas)": 67,

        # Troncal Américas
        "Portal Américas": 68, "Patio Bonito": 69, "Biblioteca Tintal": 70, "Transversal 86": 71,
        "Banderas": 72, "Mandalay": 73, "Mundo Aventura": 74, "Marsella": 75, "Pradera": 76,
        "Américas Carrera 53A": 77, "Puente Aranda": 78, "Carrera 43": 79, "Zona Industrial": 80,
        "CDS Carrera 32 (Américas)": 81, "Ricaurte (Américas)": 82, "Comuneros (Américas)": 83,

        # Troncal Calle 80
        "Portal 80": 84, "Quirigua": 85, "Carrera 90": 86, "Avenida Cali": 87, "Granja - Carrera 77": 88,
        "Minuto de Dios": 89, "Boyacá": 90, "Ferias": 91, "Avenida 68": 92, "Carrera 53": 93,
        "Carrera 47": 94, "Escuela Militar": 95, "Polo": 96,

        # Troncal Suba
        "Portal Suba": 97, "Suba - Transversal 91": 98, "Suba - Avenida Boyacá": 99, "Niza - Calle 127": 100,
        "Humedal Córdoba": 101, "Shaio": 102, "Puente Largo": 103, "Suba - Calle 100": 104,
        "Suba - Calle 95": 105, "Rionegro": 106, "San Martín": 107,

        # Troncal Calle 13
        "Portal El Dorado": 108, "Modelia": 109, "Normandía": 110, "Avenida Rojas": 111,
        "El Tiempo": 112, "Salitre El Greco": 113, "CAN": 114, "Quinta Paredes": 115,
        "Carrera 32": 116, "Ricaurte (Calle 13)": 117, "San Façon - Carrera 22": 118,
        "De La Sabana (Calle 13)": 119,

        # Troncal Caracas Sur
        "Portal Usme": 120, "Socorro": 121, "La Fiscala": 122, "Paradero Calle 85 Sur": 123,
        "Bosa San Diego": 124, "General Santander": 125, "NQS Calle 30 Sur": 126,

        # Troncal 10ª
        "Museo Nacional": 127, "San Diego": 128, "Las Nieves": 129, "San Victorino": 130,
        "Bicentenario": 131, "San Bernardo": 132,

        # Troncal Eje Ambiental
        "Avenida Jiménez (Eje Ambiental)": 133, "Museo del Oro": 134, "Vergel": 135,
        "Hortúa": 136, "Nariño": 137, "Hospital": 138,

        # Troncal Calle 6
        "Portal Tunal (Calle 6)": 139, "Calle 38A Sur": 140, "Alquería": 141, "Venecia": 142,
        "Sevillana (Calle 6)": 143, "Madelena": 144, "Perdomo": 145, "Portal Bosa": 146
    }

    # Define edges
    edges = [
        # Troncal NQS
        ("Portal Norte", "Toberin", [8, 4, 120], 0.1), ("Toberin", "Calle 161", [7, 3, 110], 0.2),
        ("Calle 161", "Mazuélen", [6, 3, 105], 0.15), ("Mazuélen", "Calle 146", [7, 4, 100], 0.2),
        ("Calle 146", "Calle 142", [6, 3, 95], 0.1), ("Calle 142", "Alcalá", [8, 4, 90], 0.25),
        ("Alcalá", "Prado", [7, 3, 85], 0.2), ("Prado", "Calle 127", [8, 4, 95], 0.3),
        ("Calle 127", "Pepe Sierra", [6, 3, 90], 0.15), ("Pepe Sierra", "Calle 106", [7, 4, 85], 0.1),
        ("Calle 106", "Calle 100", [8, 4, 100], 0.2), ("Calle 100", "Virrey", [6, 3, 95], 0.25),
        ("Virrey", "Calle 85", [7, 3, 90], 0.1), ("Calle 85", "Héroes", [8, 4, 85], 0.3),
        ("Héroes", "Calle 76", [6, 3, 80], 0.2), ("Calle 76", "Calle 72", [7, 4, 90], 0.15),
        ("Calle 72", "Flores", [6, 3, 85], 0.25), ("Flores", "Calle 63", [8, 4, 90], 0.2),
        ("Calle 63", "Calle 57", [7, 3, 95], 0.1), ("Calle 57", "Marly", [6, 3, 90], 0.3),
        ("Marly", "Calle 45", [8, 4, 100], 0.2), ("Calle 45", "Avenida 39", [7, 3, 95], 0.25),
        ("Avenida 39", "Calle 34", [6, 3, 90], 0.1), ("Calle 34", "Calle 26", [8, 4, 85], 0.3),
        ("Calle 26", "Calle 22", [7, 3, 90], 0.2), ("Calle 22", "Calle 19", [6, 3, 85], 0.15),
        ("Calle 19", "Avenida Jiménez", [8, 4, 100], 0.25), ("Avenida Jiménez", "Comuneros", [7, 3, 95], 0.3),
        ("Comuneros", "Hospitales", [6, 3, 90], 0.2), ("Hospitales", "Ricaurte", [8, 4, 85], 0.1),
        ("Ricaurte", "CDS Carrera 32", [7, 3, 90], 0.25), ("CDS Carrera 32", "Avenida NQS Calle 30 Sur", [8, 4, 95], 0.3),
        ("Avenida NQS Calle 30 Sur", "Santa Isabel", [6, 3, 90], 0.2), ("Santa Isabel", "Sevillana", [7, 3, 85], 0.15),
        ("Sevillana", "NQS Calle 38A Sur", [8, 4, 90], 0.25), ("NQS Calle 38A Sur", "NQS Calle 40 Sur", [6, 3, 85], 0.1),
        ("NQS Calle 40 Sur", "Portal Sur", [10, 5, 100], 0.3), ("Portal Sur", "Bosa", [8, 4, 95], 0.2),
        ("Bosa", "La Despensa", [7, 3, 90], 0.25), ("La Despensa", "León XIII", [6, 3, 85], 0.1),
        ("León XIII", "Terreros", [8, 4, 90], 0.3), ("Terreros", "Portal Tunal", [7, 3, 95], 0.2),
        ("Portal Tunal", "Parque", [6, 3, 90], 0.15), ("Parque", "Biblioteca", [8, 4, 85], 0.25),
        ("Biblioteca", "Portal 20 de Julio", [7, 3, 90], 0.2),

        # Troncal Caracas
        ("Portal Usme", "Molinos", [10, 5, 130], 0.4), ("Molinos", "Consuelo", [8, 4, 120], 0.3),
        ("Consuelo", "Santa Lucía", [7, 3, 115], 0.25), ("Santa Lucía", "Calle 40 Sur", [8, 4, 110], 0.2),
        ("Calle 40 Sur", "Quiroga", [6, 3, 105], 0.15), ("Quiroga", "Olaya", [7, 3, 100], 0.3),
        ("Olaya", "Restrepo", [8, 4, 95], 0.2), ("Restrepo", "Fucha", [6, 3, 90], 0.25),
        ("Fucha", "Tercer Milenio", [8, 4, 85], 0.1), ("Tercer Milenio", "Avenida Jiménez (Caracas)", [7, 3, 90], 0.3),
        ("Avenida Jiménez (Caracas)", "De La Sabana", [6, 3, 85], 0.2), ("De La Sabana", "Calle 26 (Caracas)", [8, 4, 90], 0.25),
        ("Calle 26 (Caracas)", "Calle 34 (Caracas)", [7, 3, 95], 0.1), ("Calle 34 (Caracas)", "Calle 45 (Caracas)", [8, 4, 90], 0.3),
        ("Calle 45 (Caracas)", "Marly (Caracas)", [6, 3, 85], 0.2), ("Marly (Caracas)", "Calle 57 (Caracas)", [7, 3, 90], 0.25),
        ("Calle 57 (Caracas)", "Calle 63 (Caracas)", [8, 4, 95], 0.1), ("Calle 63 (Caracas)", "Flores (Caracas)", [6, 3, 90], 0.3),
        ("Flores (Caracas)", "Calle 72 (Caracas)", [7, 3, 85], 0.2), ("Calle 72 (Caracas)", "Calle 76 (Caracas)", [8, 4, 90], 0.25),
        ("Calle 76 (Caracas)", "Héroes (Caracas)", [6, 3, 85], 0.1),
    ]

    # Add edges to the graph
    for from_station, to_station, cost, congestion in edges:
        from_idx = station_to_idx[from_station]
        to_idx = station_to_idx[to_station]
        graph.add_edge(from_idx, to_idx, cost, congestion)

    return graph, station_to_idx

def get_station_list():
    # Return list of station names (for display purposes)
    return list(create_transmilenio_map()[1].keys())

if __name__ == "__main__":
    graph, station_to_idx = create_transmilenio_map()
    print(f"Number of stations: {len(station_to_idx)}")
    print(f"Edges from Portal Norte: {graph[0]}")

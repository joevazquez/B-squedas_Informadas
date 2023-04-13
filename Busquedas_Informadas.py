import queue
import heapq
import random
from heapq import heappush, heappop
from math import inf, radians, cos, sin, asin, sqrt


def heuristic(node):
    # Coordenadas de los nodos
    latitudes = {'A': 40.4168, 'B': 40.4146, 'C': 40.4262,
                 'D': 40.4301, 'E': 40.4266, 'F': 40.4297}
    longitudes = {'A': -3.7038, 'B': -3.7015, 'C': -3.6764,
                  'D': -3.7007, 'E': -3.6974, 'F': -3.6984}
    # Distancia Haversine
    R = 6372.8  # Radio de la Tierra en kilómetros
    lat1, lon1 = radians(latitudes[node]), radians(longitudes[node])
    lat2, lon2 = radians(latitudes[goal]), radians(longitudes[goal])
    dlat = lat2 - lat1
    dlon = lon2 - lon1
    a = sin(dlat / 2)**2 + cos(lat1) * cos(lat2) * sin(dlon / 2)**2
    c = 2 * asin(sqrt(a))
    return R * c

# Función para Greedy best-first
def get_neighbors(node):
    return graph[node].keys()
# Función para Greedy best-first
def greedy(start, goal):
    queue2 = queue.PriorityQueue()
    queue2.put((heuristic(start), start))
    visited = set()
    while not queue2.empty():
        node = queue2.get()[1]

        if node == goal:
            # Si hemos llegado al nodo objetivo, podemos calcular el costo total del camino.
            return visited
        visited.add(node)

        for neighbor in get_neighbors(node):
            if neighbor not in visited:
                # Al agregar un vecino a la cola de prioridad, utilizamos la heurística para priorizar los nodos.
                queue2.put((heuristic(neighbor), neighbor))

    return None

#Función para Weighted A*
def edge_cost(edge):
    node1, node2 = edge
    return graph[node1][node2]
#Función para Weighted A*
def successors(node):
    return graph.get(node, {})

def weighted_astar(start, goal, heuristic, successors, edge_cost, w=1):
    """Weighted A* search algorithm"""
    frontier = [(heuristic(start), start, 0)]
    visited = {start: (None, 0)}
    while frontier:
        _, node, cost = heappop(frontier)
        if node == goal:
            path = [node]
            while node != start:
                node, _ = visited[node]
                path.append(node)
            return list(reversed(path))
        for successor, successor_cost in successors(node).items():
            new_cost = visited[node][1] + edge_cost((node, successor))
            if successor not in visited or new_cost < visited[successor][1]:
                visited[successor] = (node, new_cost)
                priority = new_cost + w * heuristic(successor)
                heappush(frontier, (priority, successor, new_cost))
    return []

def astar(start, goal, graph, heuristic) :
    """
    Implementación del algoritmo A* para encontrar el camino más corto desde un nodo inicial hasta un nodo objetivo
    en un grafo ponderado y dirigido.

    :param start: Nodo inicial.
    :param goal: Nodo objetivo.
    :param graph: Grafo ponderado y dirigido representado como un diccionario de diccionarios. El diccionario exterior
                  contiene los nodos del grafo como claves, y los valores son otros diccionarios que representan los
                  nodos adyacentes y sus respectivos costos. Por ejemplo: {'A': {'B': 6, 'C': 8}, 'B': {'D': 3}, ...}
    :param heuristics: Diccionario que contiene las heurísticas (en este caso, las distancias haversine) para cada nodo.
                       Las claves son los nodos del grafo y los valores son los valores de la heurística para cada nodo.
                       Por ejemplo: {'A': 2442.076702837977, 'B': 2504.893734691217, 'C': 3334.165797319547, ...}
    :return: Tupla que contiene dos diccionarios: el diccionario 'came_from', que contiene los nodos antecesores de cada
             nodo en el camino más corto desde el nodo inicial hasta el nodo objetivo, y el diccionario 'cost_so_far',
             que contiene el costo acumulado para llegar a cada nodo en el camino más corto desde el nodo inicial.
    """
    # Inicializamos los diccionarios que almacenarán los nodos antecesores y los costos acumulados
    # Creamos una cola de prioridad (heap) que almacenará los nodos por los que iremos pasando durante la búsqueda.
    # La cola de prioridad está implementada como una lista de tuplas, donde el primer elemento de la tupla es la
    # suma del costo acumulado y la heurística del nodo, y el segundo elemento es el propio nodo.
    # Los elementos de la cola de prioridad se ordenan según la suma del costo acumulado y la heurística, de manera
    # que los nodos con menor suma estén al principio de la cola.
    frontier = [(0, start)]
    came_from = {start: None}
    # Inicializamos el costo acumulado del nodo inicial a 0
    cost_so_far = {start: 0}

     # Iteramos mientras la cola de prioridad tenga elementos
    while frontier:
        _, current = heapq.heappop(frontier)
# Obtenemos el nodo actual de la cola de prioridad, es decir, el nodo con menor suma de costo acumulado y heurística
 # Si hemos llegado al nodo objetivo, terminamos la búsqueda y devolvemos los diccionarios de nodos antecesores y costos acumulados
        if current == goal:
            break
 # Iteramos sobre los nodos adyacentes al nodo actual
        for next_node, cost in graph[current].items():
           # Calculamos el costo acumulado para llegar al vecino desde el nodo actual
            # Calculamos el costo acumulado para llegar al vecino desde el nodo inicial, sumando el costo acumulado
            # para llegar al nodo actual y el costo del arco entre el nodo actual y el vecino
            new_cost = cost_so_far[current] + cost

            # Si el vecino no está en el diccionario de costos acumulados o si hemos encontrado un camino más corto
            # para llegar al vecino, actualizamos el diccionario de costos acumulados y el diccionario de nodos
            # antecesores
            if next_node not in cost_so_far or new_cost < cost_so_far[next_node]:
                cost_so_far[next_node] = new_cost
                priority = new_cost + heuristic(next_node)
                heapq.heappush(frontier, (priority, next_node))
                came_from[next_node] = current
# Devolvemos los diccionarios de nodos antecesores y costos acumulados
    return came_from, cost_so_far

def beam_search(start_state, goal_fn, expand_fn, beam_width, goal, heuristic):
    beam = [(0, start_state)]
    paths = {start_state: [start_state]}
    while True:
        next_beam = []
        for cost, state in beam:
            for child_state, child_cost in expand_fn(state):
                new_cost = cost + child_cost
                h = heuristic(child_state)
                f = new_cost + h
                if goal_fn(child_state):
                    path = paths[state] + [child_state]
                    return (new_cost, path)
                next_beam.append((f, child_state))
                paths[child_state] = paths[state] + [child_state]
        beam = heapq.nsmallest(beam_width, next_beam, key=lambda x: x[0])
        if not beam:
            return None
        print("Beam:", beam) # imprime el haz de búsqueda en cada iteración

#Función de beam
def expand_fn(state):
    return list(graph.get(state, {}).items())

def branch_and_bound_shortest_path(graph, start, end, heuristic_func):
    """
    Implementación del algoritmo de búsqueda Branch and Bound para encontrar el camino más corto desde el nodo "start" hasta el nodo "end" en un grafo representado como un diccionario de diccionarios.
    graph: diccionario de diccionarios que representa el grafo
    start: nodo de inicio
    end: nodo de destino
    heuristic_func: función heurística que estima el costo restante desde un nodo hasta el nodo de destino
    """

    # Definir la cola de prioridad (heap) y el diccionario de costos mínimos
    pq = []
    heapq.heappush(pq, (0 + heuristic_func(start), start, [start], 0))
    min_costs = {node: float('inf') for node in graph}
    min_costs[start] = 0 + heuristic_func(start)

    # Recorrer la cola de prioridad hasta encontrar el camino más corto desde el nodo "start" hasta el nodo "end"
    while pq:
        cost, node, path, total_cost = heapq.heappop(pq)

        # Si se ha encontrado el nodo de destino, devolver el camino y el costo total
        if node == end:
            return path, total_cost

        # Si el costo actual es mayor que el costo mínimo conocido, ignorar el nodo actual
        if cost > min_costs[node]:
            continue

        # Explorar los nodos adyacentes al nodo actual
        for adj_node, adj_cost in graph[node].items():
            new_cost = cost - heuristic_func(node) + adj_cost + heuristic_func(adj_node)
            new_path = path + [adj_node]
            new_total_cost = total_cost + adj_cost

            # Si se ha encontrado un camino más corto al nodo adyacente, actualizar el costo mínimo y agregar el nodo a la cola de prioridad
            if new_cost < min_costs[adj_node]:
                min_costs[adj_node] = new_cost
                heapq.heappush(pq, (new_cost, adj_node, new_path, new_total_cost))

    # Si no se encuentra un camino desde el nodo "start" hasta el nodo "end", devolver None
    return None

# Función de steepest hil climbing
def steepest_hill_climbing(graph, initial_node):
    #Verficamos que el nodo actual sea el nodo inicial y si es así se termina la función
    current_node = initial_node
    #Creamos un ciclo para que se calcule la heuristica con cada uno de los nodos recurrentes
    while True:
        #Calculamos la heuristica para el nodo actual para ver cual es su heuristica y poder compararlo con los vecinos
        current_score = heuristic(current_node)
        best_score = current_score
        best_node = current_node
        #Calcula la heuristica para cada uno de los vecinos del nodo acual y toma el que tenga 
        #una heuristica mejor para poderlo tomar como el mejor nodo
        for neighbor in graph[current_node]:
            neighbor_score = heuristic(neighbor)
            if neighbor_score > best_score:
                best_score = neighbor_score
                best_node = neighbor
        # Si es mejor valor es menor o igual que el valor del nodo actual regresa el nodo actual 
        # porque esa es la mejor opción
        if best_score <= current_score:
            return current_node
        #Sino regresa el nodo actual como el mejor valor
        current_node = best_node

#Función Stochastic hil clambing
def stochastic_hill_climbing(graph, initial_node, heuristic):
    #Verificamos si el nodo actual es el nodo de inicio si es asi el programa termina pero
    #si no es de este modo se hace una virifcación entre cada uno de los nodos vecinos de forma
    #aleatoria para poder ver el mejor camino con la mayor heuristica
    current_node = initial_node
    while True:
        #Se hace la heuristica al nodo actual para ver su mejor forma
        current_score = heuristic(current_node)
        #Se guarda en una lista los nodos la recorridos por el grafo
        neighbors = graph[current_node]
        # random.choices() debe recibir una lista y un valor de peso opcional.
        # En este caso, no se necesita un valor de peso, por lo que simplemente se
        # convierte el conjunto de vecinos en una lista antes de pasarla a
        # random.choices().
        random_neighbor = random.choices(list(neighbors))[0]
        neighbor_score = heuristic(random_neighbor)
        if neighbor_score > current_score:
            current_node = random_neighbor
        else:
            return current_node


#Función Genetic Algorithm
def genetic_algorithm(graph, population_size, num_generations, mutation_rate):
    population = generate_initial_population(population_size, graph)
    for generation in range(num_generations):
        fitness_scores = [fitness_function(chromosome, graph) for chromosome in population]
        parent1 , parent2 = select_parents(population)
        offspring = generate_offspring(parent1 , parent2)
        population = mutate_population(offspring, mutation_rate,graph)
    best_chromosome = max(population, key=lambda chromosome: fitness_function(chromosome, graph))
    return best_chromosome

#Función generate_initial_population para realizar el algoritmo de Genetic Algorithm
#Lo que hace esta función es generar una población de manera aleatoria, esta población es 
# cada uno de los nodos del grafo, los cuales representan una posible solución del problema,
# esta función toma cada nodo del grafo y los coloca de modo de un directorio para poder ver la 
# mejor solución al problema
def generate_initial_population(population_size, graph):
    population = []
    nodes = list(graph.keys())
    for i in range(population_size):
        chromosome = random.sample(nodes, len(nodes))
        population.append(chromosome)
    return population

#Función de fitness_function para realizar el algoritmo de Genetic Algorithm
"""Esta función hace la implemnetación de ver que tanta aplitud tiene cada nodo
 esto quiere decir que ve cuantos vecinos tiene cada uno de los nodos que se ecnuentran
 en el cromosoma, se busca maximizar los vecinos que son soluciones en el cromosoma"""
def fitness_function(chromosome, graph):
    fitness = 0
    for node in graph:
        for neighbor in graph[node]:
            if neighbor in chromosome:
                fitness += 1
    return fitness

#Función select_parents para realizar el algoritmo de Genetic Algorithm
"""La función genera un cruce entre dos padres la cual lo hace de la siguiente manera,
 toma un candidato y lo que hace es que selecciona un nodo de manera aleatoria para poder 
 encontrar a un candidato con el mayor fitness, ya cuando tenga el candidato con mayor fitness
 ese lo toma como padre y este procedimiento lo hace 2 veces para tener los 2 padres y lo 
 agrega a la lista de mejores andidatos la cual se guarda en padres"""
def select_parents(population):
    parent1 = random.choice(population)
    parent2 = random.choice(population)
    while parent2 == parent1 and len(population) > 1:
        parent2 = random.choice(population)
    return parent1, parent2

#Función generate_offspring para realizar el algoritmo de Genetic Algorithm
"""Esta función lo que hace es que a partir de los 2 padres crea una desendencia 
    lo hace a partir de una de las partes de cada uno de los padres para poder 
    realizar esta desecendia y esto lo hace de la siguiente manera si un número 
    aleatorio generado al azar es mayor que la tasa de cruce, se clona uno de los 
    padres como el descendiente sin realizar cruce. Si el número aleatorio es menor 
    o igual a la tasa de cruce, se selecciona un punto de cruce aleatorio entre 1 y 
    la longitud del cromosoma menos 1. Luego, se combina la primera parte del primer 
    padre con la segunda parte del segundo padre a partir del punto de cruce para 
    formar el descendiente. """
def generate_offspring(parent1, parent2):
    if len(parent1) <= 1:  # verificación de longitud de parent1
        return parent1
    crossover_point = random.randrange(1, len(parent1))
    child = parent1[:crossover_point] + parent2[crossover_point:]
    if random.random() < mutation_rate:
        mutate_population(child)
    return child
#Función mutate_population para realizar el algoritmo de Genetic Algorithm
"""Esta función lo que hace es generar una mutación en los cromosomas, esto lo hace tomando
    de forma aleatoria un nodo del cromosoma para poder realizar la mutación, esto se hace 
    con las siguientes reglas, ya que la función itera a través de cada cromosoma en la población
    y verifica si se debe aplicar una mutación. Si un número aleatorio generado al azar es menor 
    o igual a la tasa de mutación (mutation_rate), se realiza la mutación. Se crea una copia del 
    cromosoma original y se cambia un nodo aleatorio en el cromosoma por otro nodo elegido al azar 
    del grafo, el cromosoma mutado se agrega a una lista de cromosomas mutados, que se devuelve al 
    final de la función."""
def mutate_population(population, mutation_rate, graph):
    mutated_population = []
    for chromosome in population:
        mutated_chromosome = list(chromosome)
        for i in range(len(chromosome)):
            if random.random() < mutation_rate:
                mutated_chromosome[i] = random.choice(list(graph.keys()))
        mutated_population.append(mutated_chromosome)
    return mutated_population


graph = {
    'A': {'B': 1, 'C': 2},
    'B': {'A': 1, 'C': 1, 'D': 2},
    'C': {'A': 2, 'B': 1, 'D': 1},
    'D': {'B': 2, 'C': 1, 'E': 3},
    'E': {'D': 3, 'F': 1},
    'F': {'E': 1}
}



start = 'A'
goal = 'E'
beam_width = 3

# Ejecutamos el algoritmo Greedy
visited = greedy(start, goal)
if visited is not None:
    print("Resultado Greedy")
    print(f"El camino más corto desde '{start}' hasta '{goal}' es: {visited}")
else:
    print(f"No se pudo encontrar un camino válido desde '{start}' hasta '{goal}'.")

print("----------------------------------------------------------------------------")
# Ejecutamos el algoritmo A* con peso
path = weighted_astar(start, goal, heuristic, successors, edge_cost, w=1.5)
print("Resultado weighted A*")
print(path)

print("----------------------------------------------------------------------------")
# Ejecutamos el algoritmo A*
came_from, cost_so_far = astar(start, goal, graph, heuristic)

# Mostramos el resultado
if goal not in came_from:
    print(f"No se encontró un camino desde {start} hasta {goal}")
else:
 # Reconstruimos el camino desde el nodo inicial al nodo objetivo utilizando el diccionario de nodos antecesores
    path = [goal]
    node = goal
    while node != start:
        node = came_from[node]
        path.append(node)
    path.reverse()
  # Imprimimos el camino y el costo total
    print("Resultado A*")
    print(" -> ".join(node for node in path))
    #print(f"Costo total: {cost_so_far[goal_node]}")

print("----------------------------------------------------------------------------")
# Ejecutamos el algoritmo Beam
result = beam_search(start, lambda n: n == goal, expand_fn, beam_width, goal, heuristic)
print("Resultado Beam")
print(result)

print("----------------------------------------------------------------------------")
# Ejecutamos el algoritmo Branch and Bound
path, cost = branch_and_bound_shortest_path(graph, start, goal, heuristic)
print("Resultado de Branch and Bound")
print("Camino más corto:", path)
print("Costo total:", cost)

print("----------------------------------------------------------------------------")
#Ejecución del aloritmo steepest hil climbing
resultado = steepest_hill_climbing(graph, start)
print("El resultado de steepest hil climbing")
print(resultado) 

print("----------------------------------------------------------------------------")
resultado_stochastic = stochastic_hill_climbing(graph, start, heuristic)
print("El resultado de Stochastic hil clambing")
print(resultado_stochastic)

print("----------------------------------------------------------------------------")
population_size = 100
num_generations = 50
mutation_rate = 0.05
Resultado_genetic = genetic_algorithm(graph, population_size, num_generations, mutation_rate)
print("El resultado de Genetic Algorithm")
print(Resultado_genetic)
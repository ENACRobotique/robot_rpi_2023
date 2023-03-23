class Graph(object):
    
    def __init__(self):
        """Cree un graphe vide"""
        self.adj = {}

    def __repr__(self):
        """Representation en chaine de caracteres d'un graphe"""
        return '<Graph: {0.adj}>'.format(self)

    def add_node(self, u):
        """Ajoute un noeud u au graphe"""
        self.adj[u] = []

    def add_edge(self, u, v):
        """Ajoute l'arete (u, v) au graphe"""
        self.adj.setdefault(u, []).append(v)
        self.adj.setdefault(v, []).append(u)

    def size(self):
        """Renvoie le nombre de noeuds du graphe"""
        return len(self.adj)

    def nodes(self):
        """Renvoie la liste des noeuds du graphe"""
        return self.adj.keys()

    def neighbours(self, u):
        """Renvoie la liste des voisins du noeud u dans le graphe"""
        return self.adj[u]


def read_graph(file):
    g = Graph()
    neighbours = []
    points = []
    with open(file) as f:
        for line in f:
            l = line.split()
            g.add_node((l[0],float(l[1]),float(l[2]))) #nom x y 
            points.append(l[0])
            neighbours.append(l[3])
    for count, x in enumerate(neighbours):
        for y in x:
            g.add_edge(l[count],y)
    return g

file = "C:\Etude\Clubrobot\robot_rpi_2023\src\graph.txt"

g = read_graph(file)

import matplotlib.pyplot as plt

class Graph(object):
    
    def __init__(self):
        """Cree un graphe vide"""
        self.adj = {}
        self.coords = {}

    def __repr__(self):
        """Representation en chaine de caracteres d'un graphe"""
        return '<Graph: {0.adj}>'.format(self)

    def add_node(self, u,x,y):
        """Ajoute un noeud u au graphe"""
        self.adj[u] = []
        self.coords[u] = [x,y]

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
            g.add_node(l[0],float(l[1]),float(l[2])) #nom x y 
            points.append(l[0])
            nom_spliter = l[3].split(',')
            neighbours.append(nom_spliter)
    for count, x in enumerate(neighbours):
        for y in x:
            g.add_edge(points[count],y)
    return g


def print_map(graph):
    g = graph 
    plt.figure()
    for point in g.adj :
        for voisin in g.adj[point]:
            print(voisin)
            x1 = g.coords[point][0]
            y1 = g.coords[point][1]
            x2 = g.coords[voisin][0]
            y2 = g.coords[voisin][1]
            plt.plot([x1,x2],[y1,y2])
    plt.show()


file = 'src\graph.txt'
g = read_graph(file)
print_map(g)
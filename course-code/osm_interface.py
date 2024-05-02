
import osmnx as ox
import numpy as np
import matplotlib.pyplot as plt
import math
import cv2 as cv2
import numpy as np


coords = []
# lattitude longitude to cartesian coordinates
def llc(a):
    latitude = a[0]
    longitude = a[1]
    radius = 6378137 
    r_lat = math.radians(latitude)
    r_lon = math.radians(longitude)
    x = radius * math.cos(r_lat) * math.cos(r_lon)
    y = radius * math.cos(r_lat) * math.sin(r_lon)
    z = radius * math.sin(r_lat)
    k = 1
    l=1.5
    return [k*x, l*y, z]

class OSMnav():
 # Reset OSM interface with start_Point coordinates and end_point coordinates
    def ResetCoords(self,sla,slo,ela,elo):
        self.startLat = sla
        self.startLon = slo
        self.endLat = ela
        self.endLon = elo
        self.start_coords = (self.startLat, self.startLon)
        self.end_coords = (self.endLat, self.endLon)
        self.middle_coords = ((self.start_coords[0]+self.end_coords[0])/2,(self.start_coords[1]+self.end_coords[1])/2)
        self.graph = ox.graph_from_point(self.middle_coords, network_type='all_private', dist=1000)

 # Plot Graph
    def Plot(self):    
        fig, ax = ox.plot_graph(self.graph)

 # Shortest Path Query to get the path with waypoint coordinates (lat, lon)
    def ShortestPath(self):
        start_node = ox.distance.nearest_nodes(self.graph,X=self.start_coords[1],Y=self.start_coords[0])
        end_node = ox.distance.nearest_nodes(self.graph,X=self.end_coords[1],Y=self.end_coords[0])
        route = ox.shortest_path(self.graph, start_node, end_node, weight='length')
        self.route = route
        middle_coords =((self.start_coords[0]+self.end_coords[0])/2,(self.start_coords[1]+self.end_coords[1])/2)
        self.graph = ox.graph_from_point(middle_coords, network_type='all_private', dist=1000)
        #fig, ax = ox.plot_graph_route(self.graph, route, node_size=1)
        waypoints = [(self.graph.nodes[node]['y'], self.graph.nodes[node]['x']) for node in route]
        self.way_points_list = np.array([list(x) for x in waypoints ])
        return self.way_points_list
    
 # converting an arrar from [lat, lon] t0 [x,y,z]
    def getCartesian(self,waypoints):
        Cartesian =  [[llc(a)[0], llc(a)[1], llc(a)[2]] for a in waypoints]
        return Cartesian

  # save the graph to load it later
    def save_graph(self):
     fig,ax = ox.plot_graph_route(
     self.graph,self.route,
      # optionally draw on pre-existing axis
    figsize=(8, 8),  # figure size to create if ax is None
    bgcolor="w",  # background color of the plot
    node_color="b",  # color of the nodes
    node_size=15,  # size of the nodes: if 0, skip plotting them
    node_alpha=None,  # opacity of the nodes
    node_edgecolor="none",  # color of the nodes' markers' borders
    node_zorder=1,  # zorder to plot nodes: edges are always 1
    edge_color="#111111",  # color of the edges
    edge_linewidth=1,  # width of the edges: if 0, skip plotting them
    edge_alpha=None,  # opacity of the edges
    show=False,  # if True, call pyplot.show() to show the figure
    close=False,  # if True, call pyplot.close() to close the figure
    save=False,  # if True, save figure to disk at filepath
    filepath=None,  # if save is True, the path to the file
    dpi=300,  # if save is True, the resolution of saved file
    bbox=None,  # bounding box to constrain plot
)
     
     
     ax.scatter(self.v[1],self.v[0],  c='red')
     fig.savefig('plot.png')
     plt.close()

 # save image of path produced  
    def save_path(self):
     fig,ax = plt.subplots()
     ax.plot(self.path[:,0],self.path[:,1],'--o',c='red')
     ax.plot(self.curr_state[0],self.curr_state[1],'--o',c='blue')
     #plt.savefig()
     plt.close()
     fig.savefig('plot1.png')
     
 # Plot OSM Map with path highlighted
    def plotDubins(self,a,curr_state):
     figure,axes = plt.subplots(1,2)
     self.v=curr_state
     self.save_graph()
     image = plt.imread('/home/navin/webots/resources/osm_importer/OSMmod/plot.png')
     axes[1].imshow(image)
     self.path = a 
     axes[0].plot(a[:,0],a[:,1],'--o',c='red')
     self.curr_state = llc(curr_state)
     axes[0].plot(self.curr_state[0],self.curr_state[1],'--o',c='blue')
     self.save_graph()
     plt.title("Route")
     plt.show()

    
       
    
      
      





############################################
# Import modules
############################################

import time
import itertools
import numpy as np
from navground import core
from shapely.geometry import Point, LineString, Polygon, MultiPolygon
from shapely.ops import unary_union
import networkx as nx

############################################
# Graph modulation definition
############################################

class GraphsModulation(core.BehaviorModulation, name="Graphs"):

    def __init__(self):
        super().__init__()
        self.extra_lines = 0

    #############
    # pre method
    #############
    def pre(self, behavior: core.Behavior, time_step: float) -> None:
        start_time = time.time()
        l = behavior.horizon/2
        self.v = behavior.optimal_speed
        behavior.stuck, behavior.same_flow, behavior.opposite_flow = split_flows(behavior.environment_state.neighbors,behavior.orientation)
        behavior.fixed_dictionary = fixed_dic(behavior.environment_state.line_obstacles,behavior.environment_state.static_obstacles,behavior.stuck)
        behavior.FG, behavior.narrow_pairs = F_graph(behavior.fixed_dictionary, behavior.radius, behavior.safety_margin)
        behavior.narrow_area = narrow_passages(behavior.fixed_dictionary, behavior.narrow_pairs, behavior.radius, behavior.safety_margin)
        behavior.projection = trajectory_projection_1(behavior.position,l,behavior.orientation)
        behavior.ahead = ahead_neighbors(behavior.position,behavior.orientation,behavior.opposite_flow)
        behavior.state = 0
        #################
        closest_region = None
        min_distance = np.inf
        for region in behavior.narrow_area:
            if region.intersects(behavior.projection):  # Only consider regions that intersect the trajectory
                distance = region.distance(Point(behavior.position))  # Distance from agent's position
                if distance < min_distance:
                    min_distance = distance
                    closest_region = region
        if closest_region:
            if closest_region.contains(Point(behavior.position)):  # Check if the agent is inside the region
                behavior.state = 1
                pass
            else:  # The agent is approaching the region
                if behavior.ahead:
                    if any([closest_region.contains(Point(o.position)) for o in behavior.ahead]):
                        behavior.optimal_speed = 0
                        behavior.state = 1
                    elif any([closest_region.intersects(trajectory_projection_2(o.position, l, o.velocity)) for o in behavior.ahead]):
                        b_timetoenter = safe_division(closest_region.distance(Point(behavior.position)), 
                                                      np.linalg.norm(behavior.velocity))
                        o_timetoenter = [safe_division(closest_region.distance(Point(o.position)), 
                                                       np.linalg.norm(o.velocity)) for o in behavior.ahead]
                        if b_timetoenter > min(o_timetoenter):  # Opponent arrives first
                            behavior.optimal_speed = 0
                            behavior.state = 1
                        else:  # Agent enters first
                            pass  # No changes needed, agent can proceed
                    else:
                        pass  # No opponent approaching the narrow area
                else:
                    pass  # No opponents ahead

                pass  # Agent is not approaching the narrow area
                
        if behavior.state == 0:
            behavior.same_dictionary = same_dic(behavior.position,behavior.radius,behavior.same_flow)
            behavior.PAG, behavior.same_flow, behavior.same_dictionary = PA_graph(behavior.same_dictionary, behavior.same_flow,
                                                                                  behavior.radius, behavior.safety_margin)
            behavior.FPAG = FPA_graph(behavior.FG,behavior.PAG,behavior.fixed_dictionary,behavior.same_dictionary,
                                      behavior.radius,behavior.safety_margin)
            if A_blocking(behavior.FG,behavior.PAG,behavior.FPAG):
                behavior.right_count = right_count(behavior.position, behavior.orientation, behavior.same_flow)
                behavior.ahead_count = ahead_count(behavior.position, behavior.orientation, behavior.same_flow)
                behavior.optimal_speed *= 0.75**(behavior.ahead_count)
                if behavior.right_count > 0:
                    self.extra_lines += 1
                    behavior.environment_state.line_obstacles += [left_wall(behavior.position,behavior.orientation,
                                                                            2*(behavior.radius+behavior.safety_margin))]
        end_time = time.time()
        behavior.time = end_time - start_time

    #############
    # post method
    #############

    def post(self, behavior: core.Behavior, time_step: float, cmd: core.Twist2) -> core.Twist2:
        behavior.optimal_speed = self.v
        if self.extra_lines > 0:
            line_list = behavior.environment_state.line_obstacles[:-self.extra_lines]
            behavior.environment_state.line_obstacles = line_list
            self.extra_lines = 0
        return cmd

############################################
# Auxiliar functions
############################################

def split_flows(
        ns: list[core.Neighbor], orientation: float
) -> tuple[list[core.Neighbor], list[core.Neighbor]]:
    stuck_ns = []
    same_flow_ns = []
    opposing_flow_ns = []
    direction = np.array([np.cos(orientation),np.sin(orientation)])
    for n in ns:
        n_velocity = np.linalg.norm(n.velocity)
        if n_velocity < 0.01:
            stuck_ns.append(n)
        else:
            cos_theta = n.velocity.dot(direction)/n_velocity
            if cos_theta > 0.5:
                same_flow_ns.append(n)
            else:
                opposing_flow_ns.append(n)
    return stuck_ns, same_flow_ns, opposing_flow_ns

def fixed_dic(line_obstacles, static_obstacles, stuck):
    fixed_dictionary = {}
    fixed_dictionary.update({f'w{i}': LineString([tuple(wall.p1), tuple(wall.p2)]) for i, wall in enumerate(line_obstacles)})
    fixed_dictionary.update({f'o{i}': (Point(disc.position),disc.radius) for i, disc in enumerate(static_obstacles)})
    fixed_dictionary.update({f's{i}': (Point(stuck.position),stuck.radius) for i, stuck in enumerate(stuck)})
    return fixed_dictionary

def same_dic(position, radius, flow):
    return {'A': (Point(position), radius), **{f'n{i}': (Point(n.position), n.radius) for i, n in enumerate(flow)}}

def get_obstacle_and_epsilon(item):
    return (item,0) if not isinstance(item, tuple) else (item[0], item[1])

def dictionary_distance(dictionary1, key1, dictionary2, key2):
    obstacle1, epsilon1 = get_obstacle_and_epsilon(dictionary1[key1])
    obstacle2, epsilon2 = get_obstacle_and_epsilon(dictionary2[key2])
    return obstacle1.distance(obstacle2) - (epsilon1 + epsilon2)

def F_graph(dictionary,r,m):
    FG, narrow_pairs = nx.Graph(), []
    FG.add_nodes_from(dictionary.keys())
    d1, d2 = 2*r + 2*m, 4*r + 3*m
    for key1, key2 in itertools.combinations(dictionary.keys(), 2):
        distance = dictionary_distance(dictionary,key1,dictionary,key2)
        if distance < d1:
            FG.add_edge(key1,key2)
        elif distance < d2:
            narrow_pairs.append((key1,key2))
    return FG, narrow_pairs

def PA_graph(dictionary,flow,r,m):
    SAG = nx.Graph()
    SAG.add_nodes_from(dictionary.keys())
    d1 = 2*r + 2*m
    for key1, key2 in itertools.combinations(dictionary.keys(), 2):
        if dictionary_distance(dictionary, key1, dictionary, key2) < d1:
            SAG.add_edge(key1,key2)
    A_component = nx.node_connected_component(SAG, 'A')
    PAG = SAG.subgraph(A_component).copy()
    flow = [flow[int(node[1:])] for node in A_component if node.startswith('n')]
    dictionary = {key: value for key, value in dictionary.items() if key in A_component}
    return PAG, flow, dictionary
    
def FPA_graph(FG,PAG,f_dic,s_dic,r,m):
    FPAG = nx.compose(FG, PAG)
    for key1, key2 in itertools.product(f_dic.keys(), s_dic.keys()):
        if dictionary_distance(f_dic, key1, s_dic, key2) < 2*r + 2*m:
            FPAG.add_edge(key1,key2)
    return FPAG

def A_blocking(FG, PAG, FPAG):
    adjacent_nodes = {neighbor for node in PAG.nodes for neighbor in FPAG.neighbors(node)} - PAG.nodes
    return nx.number_connected_components(FG.subgraph(adjacent_nodes)) > 1    

def narrow_passages(dictionary, pairs_list, r, m):
    intersections = []
    for (key1, key2) in pairs_list:
        obstacle1, epsilon1 = get_obstacle_and_epsilon(dictionary[key1])
        obstacle2, epsilon2 = get_obstacle_and_epsilon(dictionary[key2])
        buffer_distance = 3*r + 2*m
        buffer1 = obstacle1.buffer(buffer_distance+epsilon1, quad_segs=3)
        buffer2 = obstacle2.buffer(buffer_distance+epsilon2, quad_segs=3)
        intersection = buffer1.intersection(buffer2)
        if not intersection.is_empty:
            intersections.append(intersection)
    union_of_intersections = unary_union(intersections)
    if isinstance(union_of_intersections, MultiPolygon):
        return list(union_of_intersections.geoms)
    elif isinstance(union_of_intersections, Polygon):
        return [union_of_intersections]
    else:
        return []

def trajectory_projection_1(position,l,alpha):
    x, y = position
    end_x = x + l * np.cos(alpha)
    end_y = y + l * np.sin(alpha)
    return LineString([(x, y), (end_x, end_y)])

def trajectory_projection_2(position,l,velocity):
    x, y = position
    cos, sin = velocity/np.linalg.norm(velocity)
    end_x = x + l * cos
    end_y = y + l * sin
    return LineString([(x, y), (end_x, end_y)])

def safe_division(numerator,denominator):
    return np.inf if denominator == 0 else numerator/denominator

def left_wall(position,orientation,length):
    front_point = position + np.array([np.cos(orientation), np.sin(orientation)]) * length
    left_point = position + np.array([-np.sin(orientation), np.cos(orientation)]) * length
    return core.LineSegment(front_point,left_point)

def right_count(position, orientation, flow):
    our_vector = np.array([np.cos(orientation), np.sin(orientation)])
    return sum(np.cross(our_vector, neighbor.position - position) < 0 for neighbor in flow)

def ahead_count(position, orientation, flow):
    our_vector = np.array([np.cos(orientation), np.sin(orientation)])
    return sum((neighbor.position - position).dot(our_vector) > 0 for neighbor in flow)

def ahead_neighbors(position, orientation, flow):
    our_vector = np.array([np.cos(orientation),np.sin(orientation)])
    return [neighbor for neighbor in flow if (neighbor.position - position).dot(our_vector) > 0]

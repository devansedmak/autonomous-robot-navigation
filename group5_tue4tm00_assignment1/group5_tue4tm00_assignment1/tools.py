import numpy as np
from core_proj_nav_ctrl import proj_nav_tools

def gradient(x, p, r):
    # Compute the gradient
    grad_norm = np.linalg.norm(x - p)
    if grad_norm == 0:
        return (x-p)
    else:
        return (x-p)*(1/grad_norm)

def U_repulsive(polygon, center, r):
    # Repulsive field
    nearest_points = proj_nav_tools.local_nearest(polygon, center)
   
    U =  [0.0,0.0]
    U = np.asarray(U)
    for point in nearest_points:
        U1=  [0.0,0.0]
        if np.linalg.norm(center-point) <= 2*r:
        
            if np.linalg.norm(center-point) <= (r):
                U1 =  1.4 * gradient(center, point, r)
            else:
                #U1=[2*r,2*r]-(center-point)
                U1 = gradient(center, point, r) * (1.4*np.linalg.norm(center-point)/r+2.8) 
        else:
           U1=[0.0,0.0]
        U=U+U1 # Sum all the repulsive gradients
        if np.linalg.norm(U)>=1.4:
            U=U/np.linalg.norm(U)*1.4
    return U

def safe_point(x, p, r):
    # Computes the new shifted point  
    grad = gradient(x,p,r)
    d = grad*r+p
    return d

def safe_point1(x, p, r):
    # Finds a point that is 2*r distant from the border in the direction for the gradient (x-p)
    grad = gradient(x,p,r)
    d = grad*2*r+p
    return d

def polygon_convex_interior_safe(polygon, center, r):
    # Computes the Free Local Space
    polygon = np.asarray(polygon)
    center = np.asarray(center)

    nearest_points = proj_nav_tools.local_nearest(polygon, center)

    convex_interior = polygon
    for point in nearest_points:
        point1=point
        point = safe_point(center, point, r) # Get the new point
        if np.linalg.norm(center-point) <= r:
            center1 =safe_point1(center, point1, r)
            convex_interior = proj_nav_tools.polygon_intersect_halfplane(convex_interior, point, center1-point)
        else:
            convex_interior = proj_nav_tools.polygon_intersect_halfplane(convex_interior, point, center-point)

    return convex_interior


def polygon_convex_interior_safe_new(polygon, nearest_points, center, r):
    # Computes the Free Local Space
    polygon = np.asarray(polygon)
    center = np.asarray(center)

    #nearest_points = proj_nav_tools.local_nearest(polygon, center)

    convex_interior = polygon
    for point in nearest_points:
        point1=point
        point = safe_point(center, point, r) # Get the new point
        if np.linalg.norm(center-point) <= r:
            center1 =safe_point1(center, point1, r)
            convex_interior = intersect_polygon_halfplane(convex_interior, point, center1-point)
        else:
            convex_interior = intersect_polygon_halfplane(convex_interior, point, center-point)
        convex_interior = np.array(convex_interior)
    
    num_nearest_points = nearest_points.shape[0]
    num_vertices = convex_interior.shape[0]
    #print(num_vertices)
    if (num_vertices-num_nearest_points) > 6:
        #print("ho troppi punti li tolgo")
        sampling_interval = int(num_vertices / 6)  # Ensure at least 6 points are sampled
        #print(sampling_interval)

        final_convex = []
        for i in range(6):
            final_convex.append(safe_point(center, convex_interior[i*sampling_interval, :], r))  # Select the

        """
        for point in points:
            #point = convex_interior[i*sampling_interval, :]  # Select the verte
            point = safe_point(center, point, r) # Get the new point
            if np.linalg.norm(center-point) <= r:
                center1 =safe_point1(center, point1, r)
                convex_interior = intersect_polygon_halfplane(convex_interior, point, center1-point)
            else:
                convex_interior = intersect_polygon_halfplane(convex_interior, point, center-point)
            convex_interior = np.array(convex_interior)
        """
        for point in nearest_points:
            point1=point
            point = safe_point(center, point, r) # Get the new point
            if np.linalg.norm(center-point) <= r:
                print("too close to wall")
            else:
                final_convex.append(point)
    else:
        final_convex = []
        for point in nearest_points:
            point1=point
            point = safe_point(center, point, r) # Get the new point
            if np.linalg.norm(center-point) <= r:
                print("too close to wall")
            else:
                final_convex.append(point)
        for i in range(num_vertices-num_nearest_points):
            final_convex.append(safe_point(center, convex_interior[i, :], r))  # Select the


    return np.asarray(final_convex)





def intersezione_rette(retta1, retta2):
    """
    Trova il punto di intersezione tra due rette date dalle equazioni y = m1*x + q1 e y = m2*x + q2.

    Args:
        retta1 (tuple): La prima retta nella forma (m1, q1).
        retta2 (tuple): La seconda retta nella forma (m2, q2).

    Returns:
        tuple: Punto di intersezione (x, y).
    """
    m1, q1 = retta1
    m2, q2 = retta2

    if m1 == m2:
        raise ValueError("Le rette sono parallele e non hanno intersezione.")

    # Calcola x e y dell'intersezione
    x = (q2 - q1) / (m1 - m2)
    y = m1 * x + q1
    return x, y


def trova_intersezione(poligono, centro):
    """
    Trova l'intersezione delle rette perpendicolari ai segmenti del poligono,
    passanti per i punti del poligono.

    Args:
        poligono (list of tuple): Lista di punti che rappresentano i vertici del poligono (x, y).
        centro (tuple): Punto centrale (x, y).

    Returns:
        tuple: Punto di intersezione (x, y) delle rette perpendicolari.
    """
    intersection=[]
    def retta_perpendicolare(segmento, punto):
        (x1, y1), (x2, y2) = segmento
        if x2 - x1 != 0:  # Evita divisioni per zero
            m_segmento = (y2 - y1) / (x2 - x1)
            m_perpendicolare = -1 / m_segmento
        else:  # Segmento verticale -> retta perpendicolare è orizzontale
            m_perpendicolare = 0
        
        x0, y0 = punto
        q = y0 - m_perpendicolare * x0
        return m_perpendicolare, q

    n = len(poligono)
    if n < 2:
        raise ValueError("Il poligono deve avere almeno due punti.")

    # Prendi i primi due punti e crea i segmenti
    n = len(poligono)

    for i in range(n):
        punto1 = poligono[i]
        punto2 = poligono[(i + 1) % n]  # Usa modulo per chiudere il poligono
        # Segmenti che uniscono il centro ai due punti consecutivi
        segmento1 = (punto1, centro)
        segmento2 = (punto2, centro)
        retta1 = retta_perpendicolare(segmento1, punto1)
        retta2 = retta_perpendicolare(segmento2, punto2)
        retta1 = retta_perpendicolare(segmento1, punto1)
        retta2 = retta_perpendicolare(segmento2, punto2)
        intersection.append( intersezione_rette(retta1, retta2) )

    
    return intersection



def intersect_polygon_halfplane(polygon, point, normal):
    """
    Calcola l'intersezione tra un poligono e un semipiano.

    Args:
        polygon (list of list/tuple): Vertici del poligono (ordine antiorario o orario).
        point (array-like): Punto sul semipiano.
        normal (array-like): Normale del semipiano.

    Returns:
        list: Vertici del nuovo poligono intersecato.
    """
    def is_inside(vertex):
        """Determina se un punto è all'interno del semipiano."""
        return np.dot(vertex - point, normal) >= 0

    def line_intersection(p1, p2, plane_point, plane_normal):
        """Calcola il punto d'intersezione tra un segmento e il bordo del semipiano."""
        d = p2 - p1
        t = np.dot(plane_point - p1, plane_normal) / np.dot(d, plane_normal)
        return p1 + t * d

    polygon = np.array(polygon)
    point = np.array(point)
    normal = np.array(normal)

    new_polygon = []
    for i in range(len(polygon)):
        current_vertex = polygon[i]
        next_vertex = polygon[(i + 1) % len(polygon)]

        current_inside = is_inside(current_vertex)
        next_inside = is_inside(next_vertex)

        if current_inside:
            new_polygon.append(current_vertex.tolist())

        if current_inside != next_inside:
            # Calcola il punto d'intersezione con il bordo del semipiano
            intersection = line_intersection(current_vertex, next_vertex, point, normal)
            new_polygon.append(intersection.tolist())

    return new_polygon


def intersezione_rette(retta1, retta2):
    """
    Trova il punto di intersezione tra due rette date dalle equazioni y = m1*x + q1 e y = m2*x + q2.

    Args:
        retta1 (tuple): La prima retta nella forma (m1, q1).
        retta2 (tuple): La seconda retta nella forma (m2, q2).

    Returns:
        tuple: Punto di intersezione (x, y).
    """
    m1, q1 = retta1
    m2, q2 = retta2

    if m1 == m2:
        raise ValueError("Le rette sono parallele e non hanno intersezione.")

    # Calcola x e y dell'intersezione
    x = (q2 - q1) / (m1 - m2)
    y = m1 * x + q1
    return x, y


def trova_intersezione(poligono, centro):
    """
    Trova l'intersezione delle rette perpendicolari ai segmenti del poligono,
    passanti per i punti del poligono.

    Args:
        poligono (list of tuple): Lista di punti che rappresentano i vertici del poligono (x, y).
        centro (tuple): Punto centrale (x, y).

    Returns:
        tuple: Punto di intersezione (x, y) delle rette perpendicolari.
    """
    intersection=[]
    def retta_perpendicolare(segmento, punto):
        (x1, y1), (x2, y2) = segmento
        if x2 - x1 != 0:  # Evita divisioni per zero
            m_segmento = (y2 - y1) / (x2 - x1)
            m_perpendicolare = -1 / m_segmento
        else:  # Segmento verticale -> retta perpendicolare è orizzontale
            m_perpendicolare = 0
        
        x0, y0 = punto
        q = y0 - m_perpendicolare * x0
        return m_perpendicolare, q

    n = len(poligono)
    if n < 2:
        raise ValueError("Il poligono deve avere almeno due punti.")

    # Prendi i primi due punti e crea i segmenti
    n = len(poligono)

    for i in range(n):
        punto1 = poligono[i]
        punto2 = poligono[(i + 1) % n]  # Usa modulo per chiudere il poligono
        # Segmenti che uniscono il centro ai due punti consecutivi
        segmento1 = (punto1, centro)
        segmento2 = (punto2, centro)
        retta1 = retta_perpendicolare(segmento1, punto1)
        retta2 = retta_perpendicolare(segmento2, punto2)
        retta1 = retta_perpendicolare(segmento1, punto1)
        retta2 = retta_perpendicolare(segmento2, punto2)
        intersection.append( intersezione_rette(retta1, retta2) )

    
    return intersection


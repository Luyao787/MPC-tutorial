import numpy as np
from scipy.spatial import ConvexHull

def remove_redundant_constraints(A, b, x0=None, tol=None):
    """
    Removes redundant constraints for the polyhedron Ax <= b.

    """
    # A = np.asarray(A)
    # b = np.asarray(b).flatten()
    
    if A.shape[0] != b.shape[0]:
        raise ValueError("A and b must have the same number of rows!")
    
    if tol is None:
        tol = 1e-8 * max(1, np.linalg.norm(b) / len(b))
    elif tol <= 0:
        raise ValueError("tol must be strictly positive!")
     
    # Remove zero rows in A
    Anorms = np.max(np.abs(A), axis=1)
    badrows = (Anorms == 0)
    if np.any(b[badrows] < 0):
        raise ValueError("A has infeasible trivial rows.")
        
    A = A[~badrows, :]
    b = b[~badrows]
    goodrows = np.concatenate(([0], np.where(~badrows)[0]))
        
    # Find an interior point if not supplied
    if x0 is None:
        if np.all(b > 0):
            x0 = np.zeros(A.shape[1])
        else:
            raise ValueError("Must supply an interior point!")
    else:
        x0 = np.asarray(x0).flatten()
        if x0.shape[0] != A.shape[1]:
            raise ValueError("x0 must have as many entries as A has columns.")
        if np.any(A @ x0 >= b - tol):
            raise ValueError("x0 is not in the strict interior of Ax <= b!")
            
    # Compute convex hull after projection
    btilde = b - A @ x0
    if np.any(btilde <= 0):
        print("Warning: Shifted b is not strictly positive. Convex hull may fail.")
    
    Atilde = np.vstack((np.zeros((1, A.shape[1])), A / btilde[:, np.newaxis]))
    
    hull = ConvexHull(Atilde)    
    u = np.unique(hull.vertices)    
    nr = goodrows[u]    
    h = goodrows[hull.simplices]
    
    # if nr[0] == 0:
    #     nr = nr[1:]
        
    Anr = A[nr, :]
    bnr = b[nr]
        
    return nr, Anr, bnr, h, x0

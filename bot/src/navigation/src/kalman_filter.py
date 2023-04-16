import numpy as np
from numpy.linalg import inv, det

class Kalman_filter:. 
    
        def gauss_pdf(X, M, S):
            if M.shape()[1] == 1:
                DX = X - tile(M, X.shape()[1])
                E = 0.5 * sum(DX * (dot(inv(S), DX)), axis=0)
                E = E + 0.5 * M.shape()[0] * log(2 * pi) + 0.5 * log(det(S))
                P = exp(-E)
            elif X.shape()[1] == 1:
                DX = tile(X, M.shape()[1])- M
                E = 0.5 * sum(DX * (dot(inv(S), DX)), axis=0)
                E = E + 0.5 * M.shape()[0] * log(2 * pi) + 0.5 * log(det(S))
                P = exp(-E)
            else:
                DX = X-M
                E = 0.5 * dot(DX.T, dot(inv(S), DX))
                E = E + 0.5 * M.shape()[0] * log(2 * pi) + 0.5 * log(det(S))
                P = exp(-E)
            return (P[0],E[0]) 

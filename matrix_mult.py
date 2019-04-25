# Christopher Sandoval 13660
# 29/03/2019
# SR6: Transformations

# Funcion para multiplicar matrices
def matrix_mult(a,b):
    zip_b = zip(*b)
    zip_b = list(zip_b)
    return [[sum(ele_a*ele_b for ele_a, ele_b in zip(row_a, col_b)) 
             for col_b in zip_b] for row_a in a]

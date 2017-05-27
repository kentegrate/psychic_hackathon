import numpy as np

def reject_outliers(data):
    m = 2
    u = np.mean(data)
    s = np.std(data)
    filtered = [e for e in data if (u - 2 * s < e < u + 2 * s)]
    return filtered

def calculate_mean(data):
    filtered_d = reject_outliers(data)
    return np.mean(filtered_d)

#fix me
data = [2,4,5,1,6,5,40]
print(calculate_mean(data))

import joblib
import numpy as np

#carica i modelli specificando il percorso
angle_model = joblib.load('robot4ws-gazebo-plugins/include/Modelli_DT_robot/best_bagging_angle.pkl')
mod_model = joblib.load('robot4ws-gazebo-plugins/include/Modelli_DT_robot/best_bagging_mod.pkl')
all_var_model = joblib.load('robot4ws-gazebo-plugins/include/Modelli_DT_robot/best_bagging_all_var.pkl')

def get_beta_mod(input_array):
    input_array = input_array[np.newaxis, :] #necessario se l'input è un numpy array di dim 1
    res = [angle_model.predict(input_array), mod_model.predict(input_array)]
    return np.array(res)

def get_all_var(input_array):
    input_array = input_array[np.newaxis, :] #necessario se l'input è un numpy array di dim 1
    res = all_var_model.predict(input_array)
    return res[0]

#inp = np.array([-10.21009206,  32.07101924,   0.18833877]) #array d'esempio
inp = np.array([-17.7204421984,-10.2559011355,0.19804395085]) #array d'esempio

inp2 = np.array([-17.7204421984,-10.2559011355,0.19804395085,-17.7204421956,-10.255897296,0.198044019276,-17.7204422171,-10.2557451713,0.198043956815,-17.7204422338,-10.2557651247,0.198043976603])

res = get_beta_mod(inp)
print(res)

res2 = get_all_var(inp2)
print(res2)


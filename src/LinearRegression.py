import numpy as np
from sklearn.linear_model import LinearRegression

class LR_poly:
    def __init__(self):
        '''
        feature : time
        target  : sensor data
        '''

        self.n_poly = 3
        self.feature_num = 10

        self.feature = np.zeros(self.feature_num)
        self.target = np.zeros(self.feature_num)
        self.last_time = 0
        self.coef = [0,0,0,0]

    def get_feature_list(self, data):
        if (self.feature == np.zeros(self.feature_num)).all():
            for i in range(self.feature_num):
                self.feature[i] = data + i

        else:    
            for i in range(self.feature_num - 1):
                self.feature[i] = self.feature[i+1]
            self.feature[self.feature_num - 1] = data

    def get_target_list(self, data):
        if (self.target == np.zeros(self.feature_num)).all():
            for i in range(self.feature_num):
                self.target[i] = data + i

        else:    
            for i in range(self.feature_num - 1):
                self.target[i] = self.target[i+1]
            self.target[self.feature_num - 1] = data
            

    def fit_model(self,f_data,t_data):
        self.get_feature_list(f_data)
        self.get_target_list(t_data)
        self.coef = np.polyfit(self.feature,self.target,self.n_poly)

    def estimate_data(self,TIME):
        estimated_data = 0
        for i in range(self.n_poly + 1):
            estimated_data += self.coef[i]*TIME**(self.n_poly - i)

        return estimated_data
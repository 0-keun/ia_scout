import numpy as np
from sklearn.linear_model import LinearRegression

class LR_poly:
    def __init__(self):
        self.n = 3
        self.feature_num = 10
        self.lr = LinearRegression()

        self.feature = np.zeros(self.feature_num)
        self.target = np.zeros(self.feature_num)
        self.last_time = 0

    def feature_poly(self, data):
        if (self.feature == np.zeros(self.feature_num)).all():
            for i in range(self.feature_num):
                self.feature[i] = data

        else:    
            for i in range(self.feature_num - 1):
                self.feature[i] = self.feature[i+1]
            self.feature[self.feature_num - 1] = data

        self.feature = self.feature.reshape(-1,1)

        self.feature_poly_ = np.zeros((self.feature_num,self.n))

        for i in range(self.feature_num):
            for j in range(self.n):
                element = self.feature[i,0]
                self.feature_poly_[i][j] = element**(self.n - j)

    def target_poly(self, data):
        if (self.target == np.zeros(self.feature_num)).all():
            for i in range(self.feature_num):
                self.target[i] = data

        else:    
            for i in range(self.feature_num - 1):
                self.target[i] = self.target[i+1]
            self.target[self.feature_num - 1] = data
            

    def fit_model(self,f_data,t_data):
        self.feature_poly(f_data)
        self.target_poly(t_data)
        self.lr.fit(self.feature_poly_,self.target)

    def run(self,D2_TIME):
        d = 0
        for i in range(self.n):
            d += self.lr.coef_[i]*D2_TIME**(self.n - i)
        d += self.lr.intercept_

        return d        

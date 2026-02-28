from __future__ import annotations
from pathlib import Path
import time
from typing import List

import numpy as np
import pickle
from sklearn.preprocessing import RobustScaler
from sklearn.ensemble import RandomForestRegressor


class Model:

    def __init__(self,
                 role: str,
                 _type: str = 'lr') -> None:
        self.role: str = role
        self._type: str = _type
        self.score: float = 0.0
        self.trained: bool = False

    def __call__(self, x: List[float]) -> float:
        return self.predict(x)

    def train(self,
              X: List[List[float]],
              y: List[float]) -> None:
        raise NotImplementedError

    def predict(self,
                x: List[float]) -> float:
        raise NotImplementedError

    def save(self, path_model: Path) -> None:
        raise NotImplementedError

    def load(self, path_model: Path) -> None:
        raise NotImplementedError


class ModelML(Model):

    def __init__(self,
                 role: str,
                 _type: str = 'lr') -> None:
        super().__init__(role, _type)
        self.model = RandomForestRegressor(
            n_estimators=35,
            min_samples_split=12,
            min_samples_leaf=5,
            max_features=0.6,
            max_depth=20,
            max_samples=0.8,
        )
        self.score: float = 0.0
        self.scalar: RobustScaler = RobustScaler()

    def train(self,
              X: List[List[float]],
              y: List[float]) -> None:
        _x = np.array(X)
        _x = self.scalar.fit_transform(_x)
        _t = time.time()
        _ = self.model.fit(_x, y)
        self.trained = True

    def predict(self,
                x: List[float]) -> float:
        _x = np.array([x])
        _x = self.scalar.transform(_x)
        y = self.model.predict(_x)
        return y[0]

    def save(self, path_model: Path) -> None:
        with (path_model / f'model_{self.role}_{self._type}.pkl').open('wb') as f:
            pickle.dump(self, f)
        with (path_model / f'scalar_{self.role}_{self._type}.pkl').open('wb') as f:
            pickle.dump(self.scalar, f)

    @classmethod
    def load(cls, 
             _role: str,
             _type: str,
             path_model: Path) -> Model:
        _model = ModelML(_role, _type)
        with (path_model / f'model_{_role}_{_type}.pkl').open('rb') as f:
            model = pickle.load(f)
            _model.model = model.model
            _model.trained = True
        with (path_model / f'scalar_{_role}_{_type}.pkl').open('rb') as f:
            _model.scalar = pickle.load(f)
        return _model

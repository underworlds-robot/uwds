#!/usr/bin/env python
# -*- coding: utf-8 -*-

from threading import Lock

class ConcurrentContainer(object):

    def __init__(self):
        self.__map = {}
        self.__mutex = Lock()

    def _lock(self):
        self.__mutex.acquire()

    def _unlock(self):
        self.__mutex.release()

    def update(self, ids, elements):
        self._lock()
        for el in zip(ids, elements):
            self.__map[el[0]] = el[1]
        self._unlock()

    def remove(self, ids):
        self._lock()
        removed = []
        for id in ids:
            if self.has(id):
                del self.__map[id]
                removed.append(id)
        self._unlock()
        return removed

    def delete(self, id):
        if self.has(id):
            del self.__map[id]

    def is_empty(self):
        return self.get_size() > 0

    def get_size(self):
        return len(self.__map)

    def reset(self):
        self._lock()
        self.__map.clear()
        self._unlock()

    def has(self, id):
        return id in self.ids()

    def __len__(self):
        return len(self.__map)

    def __getitem__(self, key):
        if key in self.__map:
            return self.__map[key]

    def __setitem__(self, key, item):
        self.__map[key] = item

    def __contains__(self, key):
        return self.has(key)

    def ids(self):
        return self.__map.keys()

    def __iter__(self):
        return iter(self.__map.values())

    def __next__(self):
        return next(self.__map.values())

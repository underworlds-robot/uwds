#!/usr/bin/python
# -*- coding: UTF-8 -*-
import numpy as np
import scipy.spatial.distance as distance

class GloveManager:
    def __init__(self, glove_file_path, stoplist=0, keep=[], additionnal_symbols_file_path=""):
        self.load_glove_file(glove_file_path, stoplist, additionnal_symbols_file_path)

    def load_glove_file(self, glove_file_path, stoplist=0, keep=[], additionnal_symbols_file_path=""):
        self.word_to_vector = {}
        self.word_to_index = {}
        self.index_to_word = {}

        with open(glove_file_path, 'r') as f:
            i = 0
            for line in f:
                record = line.strip().split()
                if i > stoplist:
                    insert = True
                else:
                    insert = False
                if record[0] in keep:
                    insert = True
                i += 1
                if insert:
                    token = record[0]
                    self.word_to_vector[token] = np.array(record[1:], dtype=np.float64)

            tokens = sorted(self.word_to_vector.keys())
            for idx, tok in enumerate(tokens):
                kerasIdx = idx + 1
                self.word_to_index[tok] = kerasIdx
                self.index_to_word[kerasIdx] = tok

        self.vector_dim = next(iter(self.word_to_vector.values())).shape[0]
        self.vocabulary_size = len(self.word_to_vector)

        if additionnal_symbols_file_path != "":
            symbols_to_add = []
            if additionnal_symbols_file_path is not None:
                with open(file_path, 'r') as f:
                    for line in f:
                        for symbol in line.split(" "):
                            if symbol not in self.word_vector:
                                symbols_to_add.append(symbol)

            idx = 0
            nb_symbols_to_add = len(symbols_to_add)

            for symbol in symbols_to_add:
                self.word_to_vector[symbol] = np.zeros(self.vector_dim+nb_symbols_to_add)
                self.word_to_vector[symbol][self.vector_dim+idx] = 1.0
                kerasIdx = idx + 1
                self.word_to_index[symbol] = kerasIdx
                self.index_to_word[kerasIdx] = symbol
                idx += 1

            for word, vector in self.word_to_vector.items():
                if len(vector) < self.vector_dim+nb_symbols_to_add:
                    self.word_to_vector[word] = np.append(vector, np.zeros(nb_symbols_to_add))

            self.vector_dim = next(iter(self.word_to_vector.values())).shape[0]
            self.vocabulary_size = len(self.word_to_index)

    def get_vector(self, word):
        return self.word_to_vector[word]

    def has(self, word):
        return word in self.word_to_vector

    def cosine_similarity(self, vector1, vector2):
        try:
            return (1.0 - distance.cosine(vector1, vector2))
        except ValueError:
            0.0

    def sentence_vector(self, sentence):
        ponderation = 0
        first_word = True
        vector = np.zeros(self.vector_dim)
        for word in sentence.split(" "):
            if self.has(word):
                if first_word:
                    vector = self.get_vector(word)
                    first_word = False
                else:
                    vector = np.add(vector, self.get_vector(word))
                ponderation += 1
        if not np.allclose(vector, np.zeros(self.vector_dim)):
            return np.true_divide(vector, ponderation)
        else:
            return vector

    def match(self, sentence1, sentence2):
        try:
            sentence1_vector = self.sentence_vector(sentence1)
            sentence2_vector = self.sentence_vector(sentence2)
            return self.cosine_similarity(sentence1_vector, sentence2_vector)
        except Exception as e:
            return 0.0

    def get_embedding_matrix(self):
        embedding_matrix = np.zeros((self.vocabulary_size, self.vector_dim))
        for word, i in self.word_to_index.items():
            vector = self.get_vector(word)
            embedding_matrix[i] = vector
        return embedding_matrix

    def get_evaluated_sentence(self, sentence):
        first_word = True
        eval_sentence = ""
        for word in sentence.split(" "):
            if self.has(word):
                if first_word:
                    eval_sentence = word
                    first_word = False
                else:
                    eval_sentence += " " + word
        return eval_sentence

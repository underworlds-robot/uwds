#!/bin/bash
# This script install the Glove dataset
echo "Start downloding the Glove dataset... this may take a while..."
mkdir res
cd ./res
mkdir data
cd data
mkdir glove && cd glove
wget http://nlp.stanford.edu/data/glove.6B.zip
unzip glove.6B.zip
rm glove.6B.zip
cd ..
echo "Bye bye !"

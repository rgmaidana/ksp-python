# K-Shortest-Paths python package

This package implements, in python3, methods for solving the K-Shortest-Paths problem.
The problem is a generalization of the shortest paths problem and consists in finding the "K" best paths in a graph, where K is a real definite positive integer - i.e., K = 1, 2, 3, ...
Currently only the K* algorithm from Aljazzar and Leue's [paper](https://www.sciencedirect.com/science/article/pii/S0004370211000865) is implemented.

## Installation

The package can be installed locally with PIP.
There are many ways to do this, for example:

* Clone this repository with your github client of choice.
* In a command-line, go to the downloaded repository's root directory.
* Run the local pip install command: `pip install -e .`

## Usage

The package currently implements the K* algorithm only, which can be imported and instantiated in your code.
For example:
```
from KSP import KStar
ks = KStar()
```

K* is designed to work "on-the-fly" with a successor function, exploring parts of the graph as needed.
However, this feature has not yet been implemented, and so the KStar class must be initialized with the full problem graph.
See the "kstar.py" example for details.

## To-do

* KStar:
    - Implement consistency check between rounds of Dijkstra and A*;
    - ~~Implement support for successor functions;~~
    - Implement branch-and-bound pruning (function "shouldExpand" in A*);
    - Test and validate with larger graphs.

## Acknowledgements

* Husain Aljazzar and Stefan Leue: Original algorithm;
* Sebastian Haufe: Java implementation of the K* algorithm, the [K* Java Workbench](https://www.sen.uni-konstanz.de/typo3temp/secure_downloads/88014/0/424158439e438ac66a6cdabe47f2d98198e572b9/K-star-workbench.zip);

## Contributors

* [Renan G. Maidana](http://github.com/rgmaidana)

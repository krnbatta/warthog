# Polyanya

Read the paper [here](http://www.ijcai.org/proceedings/2017/0070.pdf)!

Polyanya is a compromise-free pathfinding algorithm over navigation meshes.
This repository stores the C++ implementation used in the paper.


# Compiling

Run `make all` to compile with no optimisations and debugging information.
Run `make fast` to compile with optimisations. This has been tested with g++
6.3.1 and 7.2.0 on Arch Linux and g++ 5.4.0 on Ubuntu 16.04 (via Windows
Subsystem for Linux). This should not require any other dependencies.

These should compile two executables named `scenariorunner` and `test` in
the `bin` folder. `test` is solely used for testing, while `scenariorunner`
can run the algorithm given a mesh and a scenario.


# Usage

After compiling, you can run experiments with

```
./bin/scenariorunner [--path] [--verbose] <mesh> <scenario>
```

We have supplied some example meshes and scenarios, so you can immediately
run an example scenario with

```bash
./bin/scenariorunner ./meshes/arena.mesh ./scenarios/arena.scen
```


By default, the program prints a semicolon-separated table to stdout, with
these columns:

- `index`: The 0-based index of the given scenario in the file.
- `micro`: The wall-clock time of the entire search in microseconds.
  This includes the point-location query, but does not include reading in the
  mesh or the scenario.
- `successor_calls`: How many times the successor function was called.
- `generated`: How many nodes were generated. This includes intermediate nodes
  which aren't pushed onto the open list.
- `pushed`: How many nodes were pushed onto the open list.
- `popped`: How many nodes were popped off the open list.
- `pruned_post_pop`: How many nodes were pruned right after popping them off
  the open list. This is due to root-level pruning. (Note that we apply
  root-level pruning before we push *and* after we pop).
- `length`: The length of the path found.
- `gridcost`: The length of the grid path, given in the scenario file.

The `--path` flag overrides the output to a list of found paths, where every
point of the found path (including the start and target) is printed,
separated by spaces. An example line of output is
```
path 154; (1, 4) (15, 19) (31, 35) (44, 45)
```

The `--verbose` flag outputs extra debug information about the search as
the search progresses.

We have provided four meshes representing two maps in this repository:

- Arena, from Dragon Age Origins. The triangulation can be found in
  `meshes/arena.mesh`, while the merged triangulation can be found in
  `meshes/arena-merged.mesh`. The original grid map can be found in
  `utils/maps/arena.map`.
- Aurora, from StarCraft. The triangulation can be found in
  `meshes/aurora.mesh`, while the merged triangulation can be found in
  `meshes/aurora-merged.mesh`. The original grid map can be found in
  `utils/maps/aurora.map`.

The scenario files are provided as `scenarios/arena.scen` and
`scenarios/aurora.scen`.


# Replicating our experiments
Note that the commit we used for running the experiments is marked with the
`ijcai` tag. As of 14 Nov 2017 there have not been any changes to the actual
Polyanya implementation since then - but it is still recommended that you
check out the tag to replicate our experiments.


We started by cleaning up
[Sturtevant's Moving AI grid benchmarks](http://movingai.com/benchmarks/)
from the hog2 repository.
The cleaned up benchmark maps and scenarios can be found
[here](https://bitbucket.org/mlcui1/movingai-benchmarks).

We then converted all the grid maps into triangulations, using the
[Fade2D](http://www.geom.at/fade2d/html/)
library. These meshes are listed as "CDT" meshes in the paper. 
The tools needed to convert a grid map into a triangulation can be
found in the `utils` directory with additional documentation. However,
compiling with Fade2D may be difficult. To alleviate this, we "packed" all
of our converted triangulations using the `meshpacker` utility in the `utils`
directory, then uploaded all of the packed triangulations
[here](https://bitbucket.org/mlcui1/polyanya-triangulations-packed/).
You can "unpack" the triangulations using the `meshunpacker` utility in the
`utils` directory.

Then, we greedily merged the triangulations generated above with the
`meshmerger` utility. These meshes are listed as "M-CDT" meshes in the paper,
and generally perform the best out of the three types of meshes we generated.

Finally, we converted all the grid maps into a rectangle mesh using the
`gridmap2rects` utility. These meshes are listed as "Rect" in the paper.

We ran all benchmark sets using the optimised binaries compiled using
`make fast`.


# Implementation notes

- We defined our own mesh file format. You can view the specification in
  `utils/spec/mesh/2.txt`. Generally, we define all points in the mesh with an
  (x, y) co-ordinate, with a list of polygons the point is contained in. Then
  we define all polygons in the mesh using the previously defined points.

- We use a navigation mesh, so we only define traversable polygons. If we talk
  about "non-traversable polygons", you can imagine this as the negative space
  of the traversable polygons.

- All nodes store which polygon it is "pushing into". In the paper, this is
  called the "opposite polygon".

- The implementation needs to find which polygon the start and target lies in.
  There are efficient point location algorithms, but we use a very naive slab
  algorithm with excessive space use.

- Generating the initial search nodes is non-trivial to do in practice, but
  theoretically it is the same as in the paper: for all polygons containing
  the start, for all edges of that polygon, if that edge does not contain the
  start, generate (node, edge) as a search node.

- For our benchmarks, we use Sturtevant's Moving AI benchmarks and convert the
  grids into meshes for use in this implementation.
  When reading in scenario files, which denote which *grid square* the start
  and target are in, we arbitrarily choose the corner of the grid square with
  minimum x and y co-ordinates as our starting location (to keep this
  consistent with the
  [Java implementation of Anya](http://bitbucket.org/dharabor/pathfinding),
  Polyanya's predecessor). However, this could result in an ambiguous
  start/target location as explained two points below.

- Corners such as the center vertex of

  `.X`  
  `X.`

  are allowed on the mesh. We define that there is no straight line between
  the two Xs, and so a target can't "squeeze" through this corner.

- If the start or target lie on such a corner, it is ambiguous to what polygon
  the point should be contained in. We arbitrarily settle this
  ambiguity by shifting the start point a small amount in the positive x and
  y directions - so if this situation occurs in a converted grid map, this
  will result in a start/target location which matches the grid square given
  by the (grid) scenario.

  Note that this interpretation is arbitrary. We chose this specifically
  to match the grid square in our converted grid maps. Other interpretations
  are possible - such as taking the point to be inside **all** polygons around
  the vertex, or alternatively taking the point to not be on the mesh at all.

- Generating a final search node is done when any search node which pushes
  into the polygon containing the target is popped off the open list.
  Additionally, this final search node is **not pushed onto the open list**,
  but is instead returned immediately.

  We can do this after making the observation that the heuristic value of a
  node N pushing into the final polygon is the same as the final search node
  generated from it - as the heursitic value of node N represents a possible
  path, because the straight line from the representative point (in the
  interval, which is on the perimeter of the final polygon) to the target
  does not go through any obstacles - as the polygon is convex, and all points
  in the polygon are traversable.

  Therefore, the final search node will immediately be popped off the open
  list if we pushed it on - so we don't push it on and instead immediately
  return it.

  Note that this only works with the heuristic given. If another heuristic
  is used, this guarantee does not hold, and the final search node needs to
  be pushed onto the open list.

- Care needs to be taken when expanding a node with left interval endpoint,
  right interval endpoint and root collinear. In these cases, we take the
  interval endpoint closest to the root as the new root for successors (if this
  does not lie on a corner, we don't generate any successors). Then we generate
  all edges of the opposite polygon as successors, excluding the edge the
  interval lies on.

  Theoretically, this case can be formalised by pushing the collinear interval
  forward, resulting in "observable successor interval" of zero length.
  Therefore all the successors need to be non-observable.

- We special case successor generation when the opposite polygon is a triangle.
  If you wish to follow along with the implementation,
  [this diagram will help](https://i.imgur.com/1mIzQIY.jpg) with the variable
  names. r is the root, L/R are the endpoints of the interval, 1/2/3 are
  the vertices of the triangle, and LI/RI are the endpoints of the successors.


# License

This implementation of Polyanya is licensed under GPLv2. Please refer to
`LICENSE` for more information.

Note that several source files from Daniel Harabor's
[Warthog project](https://bitbucket.org/dharabor/pathfinding)
(also licensed under GPLv2) were used in this project with their permission.
These files are:
`helpers/cfg.cpp`, `helpers/cfg.h`, `helpers/cpool.h`, `helpers/timer.cpp` and
`helpers/timer.h`.

Additionally, Fade2D is used to generate triangulations for use with this
with this implementation. Please note that commercial use of Fade2D requires
a valid commercial license.

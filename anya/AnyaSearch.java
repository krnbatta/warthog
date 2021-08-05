package pgraph.alg;

import java.util.Hashtable;

import org.jgrapht.traverse.EuclideanDistanceHeuristic;
import org.jgrapht.traverse.Heuristic;
import org.jgrapht.util.FibonacciHeap;
import org.jgrapht.util.FibonacciHeapNode;
import pgraph.Debug;
import java.util.HashMap;
import pgraph.EventType;

import pgraph.anya.AnyaExpansionPolicy;
import pgraph.anya.AnyaNode;
import pgraph.anya.experiments.MBRunnable;
import pgraph.grid.BitpackedGrid;

// An implementation of the Anya search algorithm.
//
// @author: dharabor
// @created: 2015-04-02
//

public class AnyaSearch implements MBRunnable {

	private Debug debugger;
	private static int search_id_counter = 0;
	private AnyaExpansionPolicy expander;
	private Heuristic<AnyaNode> heuristic;
//	private Object[] pool;
//	private double[] roots;
	Hashtable<Integer, SearchNode> roots_;

	public boolean verbose = true;

	public int expanded;
	public int insertions;
	public int generated;
	public int heap_ops;
	FibonacciHeap<AnyaNode> open;

	// these can be set apriori; only used in conjunction with the
	// run method.
	public AnyaNode mb_start_;
	public AnyaNode mb_target_;
	public double mb_cost_;


	// This class holds various bits of data needed to
	// drive the search
    class SearchNode extends FibonacciHeapNode<AnyaNode>
    {
    	// parent node
    	private SearchNode parent;

    	// tracks if the node has been added to open
        public int search_id;
        public int nodeId;

        // tracks if the node has been expanded
        public boolean closed;

        SearchNode(AnyaNode vertex)
        {
        	super(vertex);
        	search_id = -1;
        	nodeId = vertex.getId();
        }

        public void reset()
        {
        	parent = null;
        	search_id = search_id_counter;
        	closed = false;
        	super.reset();
        }

        public String toString()
        {
        	return "searchnode "+this.getData().hashCode() + ";" + this.getData().toString();
        }
    }

	public AnyaSearch(AnyaExpansionPolicy expander)
	{
//		this.pool = new Object[search_space_size];
//		this.roots = new double[search_space_size];
		this.roots_ = new Hashtable<Integer, SearchNode>(65535);
		this.open = new FibonacciHeap<AnyaNode>();
		this.heuristic = expander.heuristic();
		this.expander = expander;
	}

	private void init() {
		verbose = true;
		System.out.print("verbose => ");
		System.out.println(verbose);

		search_id_counter++;
		expanded = 0;
		insertions = 0;
		generated = 0;
		heap_ops = 0;
		open.clear();
		roots_.clear();
		if (verbose) {
			debugger = new Debug();
		}
	}
    private void print_path(SearchNode current, java.io.PrintStream stream)
    {
    	if(current.parent != null)
    	{
    		print_path(current.parent, stream);
    	}
    	stream.println(current.getData().hashCode() + "; "
    			+ current.getData().root.toString()
    			+ "; g=" + current.getSecondaryKey());
    }

	public Path<AnyaNode> search(AnyaNode start, AnyaNode target)
	{
		double cost = this.search_costonly(start, target);
		// generate the path
		Path<AnyaNode> path = null;
		if(cost != -1)
		{
			SearchNode node = generate(target);
			do
			{
				path = new Path<AnyaNode>(node.getData(), path, node.getSecondaryKey());
				node = node.parent;

			}while(!(node.parent == null));
		}
		return path;
	}

	public double search_costonly(AnyaNode start, AnyaNode target)
	{
		init();
		double cost = -1;
		if(!expander.validate_instance(start, target))
		{
			return cost;
		}

		if(verbose)
		{
			debugger.startEvent(start, target);
		}

		SearchNode startNode = generate(start);

		startNode.reset();

		open.insert(startNode, heuristic.getValue(start, target), 0);

		while(!open.isEmpty())
		{
			SearchNode current = (SearchNode)open.removeMin();
			if(verbose) {
				System.out.println("expanding (f="+current.getKey()+") "+current.toString());
				debugger.expandNode(Integer.toString(current.nodeId));
			}
			expander.expand(current.getData());
			expanded++;
			heap_ops++;
			if(current.getData().interval.contains(target.root))
			{
				// found the goal
				cost = current.getKey();

				if(verbose)
				{
					print_path(current, System.err);
					System.err.println(target.toString() + "; f=" + current.getKey());
					debugger.endEvent(target, current.nodeId);
				}
				break;
			}

			// unique id for the root of the parent node
			int p_hash = expander.hash(current.getData());

			// iterate over all neighbours
			while(expander.hasNext())
			{
				AnyaNode succ = expander.next();
				SearchNode neighbour = generate(succ);

				boolean insert = true;
				int root_hash = expander.hash(succ);
				SearchNode root_rep = roots_.get(root_hash);
				double new_g_value = current.getSecondaryKey() +
						expander.step_cost();

				// Root level pruning:
				// We prune a node if its g-value is larger than the best
				// distance to its root point. In the case that the g-value
				// is equal to the best known distance, we prune only if the
				// node isn't a sibling of the node with the best distance or
				// if the node with the best distance isn't the immediate parent
				if(root_rep != null)
				{
					double root_best_g = root_rep.getSecondaryKey();
					insert = (new_g_value - root_best_g)
					   			<= BitpackedGrid.epsilon;
					boolean eq = (new_g_value - root_best_g)
							>= -BitpackedGrid.epsilon;
					if(insert && eq)
					{
						int p_rep_hash = expander.hash(root_rep.parent.getData());
						insert = (root_hash == p_hash) || (p_rep_hash == p_hash);
					}
				}

				if(insert)
				{
					neighbour.reset();
					neighbour.parent = current;
					open.insert(neighbour,
							new_g_value +
							heuristic.getValue(neighbour.getData(), target),
							new_g_value);
					roots_.put(root_hash, neighbour);

					if(verbose)
					{
						System.out.println("\tinserting with f=" + neighbour.getKey() +" (g= "+new_g_value+");" + neighbour.toString());
						debugger.updateEvent(Integer.toString(neighbour.nodeId),Integer.toString(current.nodeId),new_g_value,neighbour.getKey());
					}

					heap_ops++;
					insertions++;
				}
				else
				{
					if(verbose)
					{
						System.out.println("\told rootg: "+root_rep.getSecondaryKey());
						System.out.println("\tNOT inserting with f=" + neighbour.getKey() +" (g= "+new_g_value+");" + neighbour.toString());
					}

				}
			}
			debugger.closeNode(Integer.toString(current.nodeId));
		}
		if(verbose)
		{
			System.out.println("finishing search;");
			//TODO add finish search event!!
			debugger.outputDebugFile();
		}
		return cost;

	}

	private HashMap<String,String> nodeVariables(AnyaNode v){
		HashMap<String,String> points = new HashMap<String,String>();
		points.put("cx", Double.toString(v.root.x));
		points.put("cy", Double.toString(v.root.y));
		points.put("x1", Double.toString(v.interval.getLeft()));
		points.put("y1", Double.toString(v.interval.getRow()));
		points.put("x2", Double.toString(v.interval.getRight()));
		points.put("y2", Double.toString(v.interval.getRow()));
		return points;
	}

	private SearchNode
	generate(AnyaNode v)
	{

		SearchNode retval = new SearchNode(v);
		System.out.println(v.parentNode);

		if(verbose)
		{
			String pId = v.parentNode != null? Integer.toString(v.parentNode.getId()) :null;

			HashMap<String,String> points = nodeVariables(v);

			System.out.println(debugger);
			debugger.generateEvent(Integer.toString(v.getId()),pId,points,v.getG(),v.getF());
		}

		generated++;
		return retval;
	}

	public int getExpanded() {
		return expanded;
	}

	public void setExpanded(int expanded) {
		this.expanded = expanded;
	}

	public int getGenerated() {
		return insertions;
	}

	public void setGenerated(int generated) {
		this.insertions = generated;
	}

	public int getTouched() {
		return generated;
	}

	public void setTouched(int touched) {
		this.generated = touched;
	}

	public int getHeap_ops() {
		return heap_ops;
	}

	public void setHeap_ops(int heap_ops) {
		this.heap_ops = heap_ops;
	}

	@Override
	public void run()
	{
		mb_cost_ = this.search_costonly(mb_start_, mb_target_);
	}

	@Override
	public void cleanUp() {
		// TODO Auto-generated method stub

	}
}

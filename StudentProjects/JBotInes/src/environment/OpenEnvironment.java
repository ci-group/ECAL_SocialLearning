package environment;

import java.util.ArrayList;
import java.util.LinkedList;
import java.util.Random;

import mathutils.Vector2d;
import sensors.DistanceToBSensor;
import sensors.NearTypeBRobotSensor;
import simulation.Simulator;
import simulation.environment.Environment;
import simulation.physicalobjects.Line;
import simulation.robot.Robot;
import simulation.util.Arguments;
import utils.Cell;
import utils.Edge;
import utils.Vertex;
import controllers.RunawayController;

public class OpenEnvironment extends Environment {

	private static final String DESC_A = "type0";
	private static final String DESC_B = "type1";
	private ArrayList<Robot> typeARobots = new ArrayList<Robot>();
	private ArrayList<Robot> typeBRobots = new ArrayList<Robot>();
	private ArrayList<Robot> preyRobots = new ArrayList<Robot>();

	private int numberOfFoodSuccessfullyForaged = 0;

	private boolean preysExist;
	private boolean specialBehavior;

	private Random random;

	private double rangeB;
	private boolean connected;
	private Vector2d centerOfMass = new Vector2d(0,0);
	private LinkedList<Cell> grid = new LinkedList<Cell>();
	private static int CELLS = 50;

	private ArrayList<Vertex> vertex = new ArrayList<Vertex>();

	private Vector2d avgOrientation = new Vector2d(0,0);

	private boolean variablePreysPerSample;
	private boolean variableStepsPerSample;
	
	private int stepsInterval;
	private int startSteps;
	
    protected double wallsDistance = 100;
    private LinkedList<Line> lines = new LinkedList<Line>();

	public OpenEnvironment(Simulator simulator, Arguments arguments) {
		super(simulator, arguments);
		random = simulator.getRandom();

		preysExist = arguments.getArgumentAsIntOrSetDefault("preysexist", 0) == 1;
		specialBehavior = arguments.getArgumentAsIntOrSetDefault("specialbehavior", 0)==1; 
		variablePreysPerSample = arguments.getArgumentAsIntOrSetDefault("variablepreys", 0)==1;
		variableStepsPerSample = arguments.getArgumentAsIntOrSetDefault("variablesteps", 0)==1;
		wallsDistance = arguments.getArgumentAsDoubleOrSetDefault("wallsdistance", wallsDistance);
		
		if(variableStepsPerSample){
			startSteps = arguments.getArgumentAsIntOrSetDefault("startsteps", 2000);
			stepsInterval = arguments.getArgumentAsIntOrSetDefault("stepsinterval", 2000);
		}

		for(int i = -CELLS; i < CELLS; i++){
			for(int j = -CELLS; j < CELLS; j++){
				grid.add(new Cell(j, i));
			}
		}

		
	}

	@Override
	public void setup(Simulator simulator) {
		super.setup(simulator);

		for(Robot r: getRobots()){
			if(r.getDescription().equals(DESC_A))
				typeARobots.add(r);
			else if(r.getDescription().equals(DESC_B)) {
				typeBRobots.add(r);
				Vertex v = new Vertex(r.getId());
				vertex.add(v);
			}
		}
		
		if(variableStepsPerSample){
			steps = (int)(random.nextInt(2000)+2000);
		}
		
		//geofence points
		LinkedList<Vector2d> points = new LinkedList<Vector2d>();
		
		Vector2d upperLeft = new Vector2d(-1*wallsDistance, 1*wallsDistance);
		points.add(upperLeft);
		Vector2d upperRight = new Vector2d(1*wallsDistance, 1*wallsDistance);
		points.add(upperRight);
		Vector2d lowerRight = new Vector2d(1*wallsDistance, -1*wallsDistance);
		points.add(lowerRight);
		Vector2d lowerLeft = new Vector2d(-1*wallsDistance, -1*wallsDistance);
		points.add(lowerLeft);
		
		addLines(points, simulator);

		if(preysExist){
			Arguments programmedArgs = simulator.getArguments().get("--programmedrobots");
			int programmedRobots = programmedArgs.getArgumentAsIntOrSetDefault("numberofrobots", 1);

			if(variablePreysPerSample){
				int sample = simulator.getArguments().get("--environment").getArgumentAsInt("fitnesssample");
				int i = sample % 4;
				switch(i){
				case 0: 
					programmedRobots = 1;
					break;
				case 1: 
					programmedRobots = 5;
					break;
				case 2: 
					programmedRobots = 10;
					break;
				case 3: 
					programmedRobots = 20;
					break;
				}
			}	
			
			for(int i = 0; i < programmedRobots; i++) {
				Robot prey = Robot.getRobot(simulator, programmedArgs);
				prey.setController(new RunawayController(simulator, prey, programmedArgs, specialBehavior));
				prey.setPosition(newRandomPosition());
				addRobot(prey);
				preyRobots.add(prey);
			}
		}
		//		typeBRobots.get(0).setPosition(0, 2);
		//		typeBRobots.get(1).setPosition(0, 2.1);
	}

	public Vector2d newRandomPosition() {		
		double radius = random.nextDouble()*47 + 3; //posiciona presas ente 3 e 50
		double angle = random.nextDouble()*2*Math.PI;
		return new Vector2d(radius*Math.cos(angle) + centerOfMass.x, radius*Math.sin(angle) + centerOfMass.y);
	}

	private void updateCenterOfMass() {
		double totalX = 0;
		double totalY = 0;

		for(Robot b: typeBRobots){
			totalX += b.getPosition().x;
			totalY += b.getPosition().y;
		}
		for(Robot a: typeARobots){
			totalX += a.getPosition().x;
			totalY += a.getPosition().y;
		}
		double centerX = totalX/(typeARobots.size() + typeBRobots.size());
		double centerY = totalY/(typeARobots.size() + typeBRobots.size());

		centerOfMass = new Vector2d(centerX, centerY);	
	}

//	private void updateAvgOrientation() {
//		double x = 0.0;
//		double y = 0.0;
//
//		for(Robot r: robots) {
//			if(!r.getDescription().equals("prey")){
//				x += Math.cos(r.getOrientation());
//				y += Math.sin(r.getOrientation());
//			}
//		}jj
//
//		avgOrientation = new Vector2d(x, y);
//
//	}

	@Override
	public void update(double time) {
		updateConnected();
	}

	private void updateConnected() {
		updateCenterOfMass();
		//updateAvgOrientation();

		DistanceToBSensor sensorB = (DistanceToBSensor) typeBRobots.get(0).getSensorByType(DistanceToBSensor.class);
		rangeB = sensorB.getRange();

		ArrayList<Edge> edges = new ArrayList<Edge>();

		for(Robot b: typeBRobots) {
			for(Robot r: typeBRobots) {
				if(!r.equals(b) && r.getPosition().distanceTo(b.getPosition()) <= rangeB - r.getRadius()) {
					Edge edge = null;
					if(r.getId() < b.getId())
						edge = new Edge(new Vertex(r.getId()), new Vertex(b.getId()));
					else 
						edge = new Edge(new Vertex(b.getId()), new Vertex(r.getId()));
					if(!edges.contains(edge))
						edges.add(edge);
				}
			}
		}

		connected = true;

		for(Vertex v: vertex){
			LinkedList<Edge> vertexEdges = new LinkedList<Edge>(); 
			for(Edge edge: edges) {
				if(edge.contains(v))
					vertexEdges.add(edge);
			}
			v.setEdges(vertexEdges);
			if(vertexEdges.isEmpty())
				connected = false;
		}

		int numberOfNodes = vertex.size();
		LinkedList<Vertex> closed = new LinkedList<Vertex>();

		LinkedList<Vertex> openList = new LinkedList<Vertex>();
		openList.add(vertex.get(0));

		while(closed.size() != numberOfNodes && !openList.isEmpty() && connected){ 
			while(openList.getFirst().isClosed()) {
				openList.pop();
			}
			Vertex closingVertex = openList.pop();
			closed.add(closingVertex);

			//for each non closed vertex with an edge from closingVertex
			for(Edge edge: closingVertex.getEdges()){
				int index = vertex.indexOf(edge.getNeighbour(closingVertex));
				if(!closed.contains(vertex.get(index)))
					openList.push(vertex.get(index));
			}
		}

		for(Robot robot: typeARobots){
			if(robot.getSensorByType(NearTypeBRobotSensor.class).getSensorReading(0) != 1)
				connected = false;
		}

		if(closed.size() != numberOfNodes) 
			connected = false;


		if(preysExist){
			for(Robot r: typeARobots){
				for(Robot prey: preyRobots){
					double distance = prey.getPosition().distanceTo(r.getPosition());
					if(distance < (r.getRadius() + 0.05)) {
						if(connected)
							numberOfFoodSuccessfullyForaged++;
						prey.teleportTo(new Vector2d(120,120));
					}
				}

			}
		}
	}
	
	protected void addLines(LinkedList<Vector2d> waypoints, Simulator simulator) {

        for (int i = 1; i < waypoints.size(); i++) {

            Vector2d va = waypoints.get(i - 1);
            Vector2d vb = waypoints.get(i);

            simulation.physicalobjects.Line l = new simulation.physicalobjects.Line(simulator, "line" + i, va.getX(), va.getY(), vb.getX(), vb.getY());
            addObject(l);
        }

        Vector2d va = waypoints.get(waypoints.size() - 1);
        Vector2d vb = waypoints.get(0);

        Line l = new Line(simulator, "line0", va.getX(), va.getY(), vb.getX(), vb.getY());
        addObject(l);
    }

	public Vector2d getCenterOfMass() {
		return centerOfMass;
	}

	public int getNumberOfFoodSuccessfullyForaged() {
		return numberOfFoodSuccessfullyForaged;
	}

	public ArrayList<Robot> getTypeARobots() {
		return typeARobots;
	}

	public ArrayList<Robot> getTypeBRobots() {
		return typeBRobots;
	}

	public ArrayList<Robot> getPreyRobots() {
		return preyRobots;
	}

	public boolean isConnected() {
		return connected;
	}

	public LinkedList<Cell> getGrid() {
		return grid;
	}

	public Vector2d getAvgOrientation() {
		return avgOrientation;
	}
}

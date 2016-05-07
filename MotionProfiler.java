import java.text.DecimalFormat;
import java.util.Hashtable;

public class MotionProfiler {

	private double maxVel;  //units/s
	private double maxAccel;//units/s^2
	private double timeStep;//s

	private Hashtable<String, Double[]> profile;
	
	public MotionProfiler(double maxVel, double maxAccel, double timeStep){
		this.maxVel = maxVel;
		this.maxAccel = maxAccel;
		this.timeStep = timeStep;
		profile = new Hashtable<String, Double[]>();
	}
	
	//startVel - units/s
	//target - units
	public double calc(double startVel, double target){
		double timeForAccel = (maxVel - startVel) / maxAccel;
		double timeForDecel = (maxVel) / maxAccel;
		double distanceForMaxAccel = (Math.pow(maxVel, 2) - Math.pow(startVel, 2)) / (2 * maxAccel);
		double distanceForMaxDecel = Math.pow(maxVel, 2) / (2 * maxAccel);
		double distanceForInitDecel = Math.pow(startVel, 2) / (2 * maxAccel);
		double totalTime = 0.0;
		double positionTracker = 0;
		double currentSpeed = startVel;
		
		boolean initialOvershoot = ((currentSpeed > 0 && target > 0) || (currentSpeed < 0 && target < 0)) && Math.abs(distanceForInitDecel) > Math.abs(target);
		boolean initialReverseVel = currentSpeed < 0;
		
		if(initialReverseVel){
			double initialTime = 0 - currentSpeed / maxAccel;
			double initialDistance = Math.pow(currentSpeed, 2) / (2 * maxAccel);
			addSegment(currentSpeed, 0, initialTime, totalTime, positionTracker);
			target += initialDistance;
			positionTracker -= initialDistance;
			totalTime += initialTime;
			currentSpeed = 0;
		}
		else if(initialOvershoot){
			double decelTime = currentSpeed / maxAccel;
			double decelDist = Math.pow(currentSpeed,  2) / (2 * maxAccel);
			addSegment(currentSpeed, 0, decelTime, totalTime, positionTracker);
			target -= decelDist;
			positionTracker += decelDist;
			totalTime += decelTime;
			currentSpeed = 0;
		}
		
		if(distanceForMaxAccel + distanceForMaxDecel <= target){
			timeForAccel = (maxVel - currentSpeed) / maxAccel;
			distanceForMaxAccel = (Math.pow(maxVel, 2) - Math.pow(currentSpeed, 2)) / (2 * maxAccel);
			double coastDist = target - distanceForMaxAccel - distanceForMaxDecel;
			double coastTime = coastDist / maxVel;
			
			addSegment(currentSpeed, maxVel, timeForAccel, totalTime, positionTracker);
			totalTime += timeForAccel + 0.1;
			positionTracker += distanceForMaxAccel;
			target -= distanceForMaxAccel;
			currentSpeed = maxVel;
			
			addSegment(maxVel, maxVel, coastTime, totalTime, positionTracker);
			totalTime += coastTime + 0.1;
			positionTracker += coastDist;
			target -= coastDist;
			currentSpeed = maxVel;

			addSegment(maxVel, 0, timeForDecel, totalTime, positionTracker);
			totalTime += timeForDecel + 0.1;
			positionTracker += distanceForMaxDecel;
			target -= distanceForMaxDecel;
			currentSpeed = 0;
			
		}
		else{
			double timeForFinalDecel = currentSpeed / maxAccel;
			double distanceForFinalDecel = Math.pow(currentSpeed, 2) / (2 * maxAccel);
			
			double distanceForInitial = (target - distanceForFinalDecel) / 2;
			double maxReachableVel = Math.sqrt(Math.pow(currentSpeed, 2) + 2 * maxAccel * Math.abs(distanceForInitial));
			double time_2 = (maxReachableVel - currentSpeed) / maxAccel;
			
			addSegment(currentSpeed, (target < 0 ? -1 : 1) * maxReachableVel, time_2, totalTime, positionTracker);
			totalTime += time_2;
			positionTracker += distanceForInitial;
			target -= distanceForInitial;
			currentSpeed = (target < 0 ? -1 : 1) * maxReachableVel;
			
			addSegment(currentSpeed, 0, time_2 + timeForFinalDecel, totalTime, positionTracker);
			totalTime += time_2 + timeForFinalDecel;
			positionTracker += distanceForInitial + distanceForFinalDecel;
			target -= distanceForInitial + distanceForFinalDecel;
		}
		
		return totalTime;
	}
	
	private void addSegment(double startVel, double endVel, double duration, double startTime, double startPosition){
		double segmentAccel = (endVel - startVel) / duration;
		double lastPos = startPosition;
		double lastVel = 0;//startVel;
		for(int i = 1; i < duration / this.timeStep; i++){
			Double[] data = new Double[4];
			double timeElapsed = (i * this.timeStep);
			data[0] = startTime + timeElapsed;
			data[1] = lastPos + timeStep * lastVel;
			data[2] = startVel + timeElapsed * segmentAccel;
			data[3] = segmentAccel;
			this.profile.put(getStringForVal(startTime + timeElapsed), data);
			lastPos = data[1];
			lastVel = data[2];
		}
		Double[] data = new Double[4];
		data[0] = startTime + duration;
		data[1] = lastPos + timeStep * lastVel;
		data[2] = endVel;
		data[3] = 0.0;
		this.profile.put(getStringForVal(startTime + duration), data);
	}
	
	public double getValAtTime(double time, Hashtable<String, Double[]> source){
		if(time > source.size() * this.timeStep){
			return 0.0;
		}
		else{
			return source.get(getStringForVal(time))[1];
		}
	}

	
	public String getStringForVal(double value){
		return new DecimalFormat("0.00").format(value);
	}
	
	public Double[] getMotionAtTime(double time){
		Double[] step = this.profile.get(getStringForVal(time));
		return step;
	}
	
	public void printFinalProfile(double totalTime){
		for(int i = 0; i < (int)Math.ceil(totalTime / timeStep); i++){
			String iter = getStringForVal(i * timeStep);
			Double[] data =  this.profile.get(iter);
			if(data == null) continue;
			for(int j = 0; j < data.length; j++){
				System.out.print(getStringForVal(data[j]));
				System.out.print(", ");
			}
			System.out.print("\n");
		}
	}

	public static void main(String[] args){
		MotionProfiler profiler = new MotionProfiler(10, 9, 0.01);  //Drivetrain
//		MotionProfiler profiler = new MotionProfiler(70, 60, 0.01, 50); //Arm
		double time = profiler.calc(0, 16);
		profiler.printFinalProfile(time);
	}

}

package Team4450.Robot8;

import Team4450.Lib.*;
import edu.wpi.first.wpilibj.image.*;
//import edu.wpi.first.wpilibj.image.NIVision.MeasurementType;

public class Vision2014
{
	//Threshold		threshold;
	private final	Robot		robot;

	public int					centerX;
	public int					centerY;
	public double				distance;

	public static final int X_IMAGE_RES = 640;
	public static final int Y_IMAGE_RES = 480;

	// VIEW_ANGLE 49		Axis M1013
	// VIEW_ANGLE 41.7	Axis 206 camera
	// VIEW_ANGLE 37.4  Axis M1011 camera
	public static final double VIEW_ANGLE = 49;
	public static final double PI = 3.141592653;
	public static final int RECTANGULARITY_LIMIT = 40;
	public static final int ASPECT_RATIO_LIMIT = 55;
	public static final int TAPE_WIDTH_LIMIT = 50;
	public static final int VERTICAL_SCORE_LIMIT = 50;
	public static final int LR_SCORE_LIMIT = 50;
	public static final int AREA_MINIMUM = 150;
	public static final int MAX_PARTICLES = 8;
	public static final int	NoTarget = 0;
	public static final int	TargetFound = 1;
	public static final int	HotTargetFound = 2;
   
	class Scores
	{
		double rectangularity;
		double aspectRatioVertical;
		double aspectRatioHorizontal;
	}
    
	class TargetReport 
	{
		int verticalIndex;
		int horizontalIndex;
		boolean Hot;
		double totalScore;
		double leftScore;
		double rightScore;
		double tapeWidthScore;
		double verticalScore;
	};

	Vision2014(Robot robot)
	{
		Util.consoleLog("MyVision2014.constructor");
    
		this.robot = robot;
	}

	public void dispose()
	{
		Util.consoleLog("MyVision2014.destructor");
	}

	public int locateTarget()
	{
		int		result = NoTarget;
		int[] 	verticalTargets = new int[MAX_PARTICLES];
		int[] 	horizontalTargets = new int[MAX_PARTICLES];
		int		verticalTargetCount;
		int		horizontalTargetCount;

		//CriteriaCollection	cc;      // the criteria for doing the particle filter operation
  		
		TargetReport	target = new TargetReport();

		ColorImage		image;
		
		//  HSV threshold criteria, ranges are in that order ie. Hue is 60-100
		//	Threshold		threshold(60, 100, 90, 255, 20, 255);
		//	Threshold		threshold(60, 137, 70, 255, 113, 255);
    
		//Particle filter criteria, used to filter out small particles

		//cc = new CriteriaCollection();      // create the criteria for the particle filter
		//cc.addCriteria(MeasurementType.IMAQ_MT_AREA, AREA_MINIMUM, 65535, false);
    
		Util.consoleLog();
    
		centerX = centerY = 0;
		distance = 0.0;
    
		/**
		 * Do the image capture with the camera and apply the algorithm described above. This
		 * sample will either get images from the camera or from an image file stored in the top
		 * level directory in the flash memory on the cRIO. The file name in this case is "testImage.jpg"
		 */
    
		while (!robot.camera.isFreshImage())
		{
			Util.consoleLog("wait for image");
		}

		try
		{
			image = robot.camera.getImage();

			// Use alliance and start position to select different sample camera images loaded into cRio memory
			// for testing.
    
	//	if (myRobot->alliance == 0 && myRobot->startLocation == 1)
	//		image = new RGBImage("/Center.jpg");		// get the sample image from the cRIO flash
	//	if (myRobot->alliance == 0 && myRobot->startLocation == 2)
	//		image = new RGBImage("/Center_18ft_On.jpg");		// get the sample image from the cRIO flash
	//	if (myRobot->alliance == 0 && myRobot->startLocation == 3)
	//		image = new RGBImage("/Right_18ft_On.jpg");		// get the sample image from the cRIO flash
	//	if (myRobot->alliance == 1 && myRobot->startLocation == 1)
	//		image = new RGBImage("/Center_27ft_Off.jpg");		// get the sample image from the cRIO flash
	//	if (myRobot->alliance == 1 && myRobot->startLocation == 2)
	//		image = new RGBImage("/10ft_Left.jpg");		// get the sample image from the cRIO flash
	//	if (myRobot->alliance == 1 && myRobot->startLocation == 3)
	//		image = new RGBImage("/10ft_Left_Off.jpg");		// get the sample image from the cRIO flash

			Util.consoleLog("1");
	
			image.write("/camera.bmp");

			Util.consoleLog("1.01");

			//BinaryImage thresholdImage = image.ThresholdHSV(threshold); // get just the green target pixels

			// keep only green objects
			//BinaryImage thresholdImage = image.thresholdHSV(105, 137, 230, 255, 133, 183);
			BinaryImage thresholdImage = image.thresholdRGB(0, 45, 25, 255, 0, 47);

			thresholdImage.write("/threshold.bmp");
			
			Util.consoleLog("1.1");

			BinaryImage convexHullImage = thresholdImage.convexHull(false); // fill in partial and full rectangles
			
			convexHullImage.write("/convexHull.bmp");

			Util.consoleLog("1.2");

			BinaryImage filteredImage = null; //= convexHullImage.particleFilter(cc);		//Remove small particles
// resolve this duplicate definition by looking at myrobot6.
			//BinaryImage filteredImage //= thresholdImage.particleFilter(cc);    // filter out small particles

			filteredImage.write("/filtered.bmp");
			
			Util.consoleLog("1.3");

			//iterate through each particle and score to see if it is a target

			Scores scores[] = new Scores[filteredImage.getNumberParticles()];
			Util.consoleLog("1.3.1");
			horizontalTargetCount = verticalTargetCount = 0;
				
			Util.consoleLog("1.3.2 np=%d" + filteredImage.getNumberParticles());

			if(filteredImage.getNumberParticles() > 0)
			{
				Util.consoleLog("1.4");
				
				for (int i = 0; i < MAX_PARTICLES && i < filteredImage.getNumberParticles(); i++) 
				{
					Util.consoleLog("MyVision2104.locateTarget-1.4.1");
					ParticleAnalysisReport report = filteredImage.getParticleAnalysisReport(i);
					scores[i] = new Scores();

					//Score each particle on rectangularity and aspect ratio
					scores[i].rectangularity = scoreRectangularity(report);
					scores[i].aspectRatioVertical = scoreAspectRatio(filteredImage, report, i, true);
					scores[i].aspectRatioHorizontal = scoreAspectRatio(filteredImage, report, i, false);			

					//Check if the particle is a horizontal target, if not, check if it's a vertical target
					Util.consoleLog("MyVision2104.locateTarget-1.4.2");
					if(scoreCompare(scores[i], false))
					{
						Util.consoleLog("particle: " + i + " is a Horizontal Target centerX: " + report.center_mass_x + " centerY: "
								+ report.center_mass_y);

						horizontalTargets[horizontalTargetCount++] = i; //Add particle to target array and increment count
					} 
					else if (scoreCompare(scores[i], true))
					{
						Util.consoleLog("particle: " + i + " is a Vertical Target centerX: " + report.center_mass_x + " centerY: "
								+ report.center_mass_y);

						verticalTargets[verticalTargetCount++] = i;  //Add particle to target array and increment count
					} 
					else
					{
						Util.consoleLog("particle: " + i + " is not a Target centerX: " + report.center_mass_x + " centerY: "
								+ report.center_mass_y);
					}

					Util.consoleLog("rect: " + scores[i].rectangularity + " ARHoriz: " + scores[i].aspectRatioHorizontal);

					Util.consoleLog("ARVert: " + scores[i].aspectRatioVertical);	
				}

				Util.consoleLog("1.5");

				//Zero out scores and set verticalIndex to first target in case there are no horizontal targets
				target.totalScore = target.leftScore = target.rightScore = target.tapeWidthScore = target.verticalScore = 0;
				target.verticalIndex = verticalTargets[0];

				for (int i = 0; i < verticalTargetCount; i++)
				{
					ParticleAnalysisReport verticalReport = filteredImage.getParticleAnalysisReport(verticalTargets[i]);

					for (int j = 0; j < horizontalTargetCount; j++)
					{
						ParticleAnalysisReport horizontalReport = filteredImage.getParticleAnalysisReport(horizontalTargets[j]);
						double horizWidth = 0, horizHeight = 0, vertWidth = 0, leftScore, rightScore, tapeWidthScore, verticalScore, total;

						//Measure equivalent rectangle sides for use in score calculation
//						horizWidth = NIVision.MeasureParticle(filteredImage.image, horizontalTargets[j], false, 
//								MeasurementType.IMAQ_MT_EQUIVALENT_RECT_LONG_SIDE);
//						vertWidth = NIVision.MeasureParticle(filteredImage.image, verticalTargets[i], false, 
//								MeasurementType.IMAQ_MT_EQUIVALENT_RECT_SHORT_SIDE);
//						horizHeight = NIVision.MeasureParticle(filteredImage.image, horizontalTargets[j], false, 
//								MeasurementType.IMAQ_MT_EQUIVALENT_RECT_SHORT_SIDE);

						//Determine if the horizontal target is in the expected location to the left of the vertical target
						leftScore = ratioToScore(1.2*(verticalReport.boundingRectLeft - horizontalReport.center_mass_x)/horizWidth);
						//Determine if the horizontal target is in the expected location to the right of the  vertical target
						rightScore = ratioToScore(1.2*(horizontalReport.center_mass_x - verticalReport.boundingRectLeft 
								- verticalReport.boundingRectWidth)/horizWidth);
						//Determine if the width of the tape on the two targets appears to be the same
						tapeWidthScore = ratioToScore(vertWidth/horizHeight);
						//Determine if the vertical location of the horizontal target appears to be correct
						verticalScore = ratioToScore(1-(verticalReport.boundingRectTop - horizontalReport.center_mass_y) 
								/ (4*horizHeight));
						
						total = leftScore > rightScore ? leftScore:rightScore;
						total += tapeWidthScore + verticalScore;

						//If the target is the best detected so far store the information about it
						if(total > target.totalScore)
						{
							target.horizontalIndex = horizontalTargets[j];
							target.verticalIndex = verticalTargets[i];
							target.totalScore = total;
							target.leftScore = leftScore;
							target.rightScore = rightScore;
							target.tapeWidthScore = tapeWidthScore;
							target.verticalScore = verticalScore;
						}
					}

					//Determine if the best target is a Hot target
					target.Hot = hotOrNot(target);
				}

				if(verticalTargetCount > 0)
				{
					//Information about the target is contained in the "target" structure
					//To get measurement information such as sizes or locations use the
					//horizontal or vertical index to get the particle report as shown below
					ParticleAnalysisReport distanceReport = filteredImage.getParticleAnalysisReport(target.verticalIndex);
					
					distance = computeDistance(filteredImage, distanceReport, target.verticalIndex);

					if(target.Hot)
					{
						Util.consoleLog("Hot target located, distance: " + distance);
						result = HotTargetFound;
					}
					else
					{
						Util.consoleLog("Cold target present, distance: " + distance);
						result = TargetFound;
					}
				}
			}

			/**
			 * all images in Java must be freed after they are used since they are allocated out
			 * of C data structures. Not calling free() will cause the memory to accumulate over
			 * each pass of this loop.
			 */
	
			Util.consoleLog("end result=%d" + result);
		
			filteredImage.free();
			thresholdImage.free();
			convexHullImage.free();
			image.free();

			return result;
		}
		catch (Throwable e)
		{
			e.printStackTrace(Util.logPrintStream);
			return NoTarget;
		}
	}

	/**
	 * Computes the estimated distance to a target using the height of the particle in the image. For more information and
	 * graphics
	 * showing the math behind this approach see the Vision Processing section of the ScreenStepsLive documentation.
	 * 
	 * @param image The image to use for measuring the particle estimated rectangle
	 * @param report The Particle Analysis Report for the particle
	 * @param outer True if the particle should be treated as an outer target, false to treat it as a center target
	 * @return The estimated distance to the target in Inches.
	 */
	double computeDistance (BinaryImage image, ParticleAnalysisReport report, int particleNumber) 
			throws NIVisionException 
	{
		double rectLong = 0, height;
		int targetHeight;

//		rectLong = NIVision.MeasureParticle(image.image, particleNumber, false, 
//				MeasurementType.IMAQ_MT_EQUIVALENT_RECT_LONG_SIDE);

		//using the smaller of the estimated rectangle long side and the bounding rectangle height results in better 
		//performance on skewed rectangles

		height = Math.min(report.boundingRectHeight, rectLong);
		targetHeight = 32;

		return Y_IMAGE_RES * targetHeight / (height * 12 * 2 * Math.tan(VIEW_ANGLE*Math.PI/(180*2)));
	}

	/**
	 * Computes a score (0-100) comparing the aspect ratio to the ideal aspect ratio for the target. This method uses
	 * the equivalent rectangle sides to determine aspect ratio as it performs better as the target gets skewed 
	 * by moving to the left or right. The equivalent rectangle is the rectangle with sides x and y where particle
	 * area= x*y and particle perimeter= 2x+2y
	 * 
	 * @param image The image containing the particle to score, needed to perform additional measurements
	 * @param report The Particle Analysis Report for the particle, used for the width, height, and particle number
	 * @param outer	Indicates whether the particle aspect ratio should be compared to the ratio for the inner target
	 * or the outer
	 * @return The aspect ratio score (0-100)
	 */
	public double scoreAspectRatio(BinaryImage image, ParticleAnalysisReport report, int particleNumber, 
			boolean vertical) throws NIVisionException
	{
		double rectLong = 0, rectShort = 0, aspectRatio, idealAspectRatio;

//		rectLong = NIVision.MeasureParticle(image.image, particleNumber, false, 
//				MeasurementType.IMAQ_MT_EQUIVALENT_RECT_LONG_SIDE);
//		rectShort = NIVision.MeasureParticle(image.image, particleNumber, false, 
//				MeasurementType.IMAQ_MT_EQUIVALENT_RECT_SHORT_SIDE);
		
		idealAspectRatio = vertical ? (4.0/32) : (23.5/4);	
		//Vertical reflector 4" wide x 32" tall, horizontal 23.5" wide x 4" tall

		//Divide width by height to measure aspect ratio
		if(report.boundingRectWidth > report.boundingRectHeight)
		{
				//particle is wider than it is tall, divide long by short
				aspectRatio = ratioToScore((rectLong/rectShort)/idealAspectRatio);
		} else 
		{
				//particle is taller than it is wide, divide short by long
				aspectRatio = ratioToScore((rectShort/rectLong)/idealAspectRatio);
		}
		
		return aspectRatio;
	}

	/**
	 * Compares scores to defined limits and returns true if the particle appears to be a target
	 * 
	 * @param scores The structure containing the scores to compare
	 * @param outer True if the particle should be treated as an outer target, false to treat it as a center target
	 * 
	 * @return True if the particle meets all limits, false otherwise
	 */
	boolean scoreCompare(Scores scores, boolean vertical)
	{
		boolean isTarget = true;

		isTarget &= scores.rectangularity > RECTANGULARITY_LIMIT;
		
		if(vertical)
			isTarget &= scores.aspectRatioVertical > ASPECT_RATIO_LIMIT;
		else
			isTarget &= scores.aspectRatioHorizontal > ASPECT_RATIO_LIMIT;
		
		return isTarget;
	}

	/**
	 * Computes a score (0-100) estimating how rectangular the particle is by comparing the area of the particle
	 * to the area of the bounding box surrounding it. A perfect rectangle would cover the entire bounding box.
	 * 
	 * @param report The Particle Analysis Report for the particle to score
	 * @return The rectangularity score (0-100)
	 */
	double scoreRectangularity(ParticleAnalysisReport report)
	{
		if(report.boundingRectWidth*report.boundingRectHeight != 0)
			return 100*report.particleArea / (report.boundingRectWidth*report.boundingRectHeight);
		else
			return 0;
	}
    
  /**
	* Converts a ratio with ideal value of 1 to a score. The resulting function is piecewise
	* linear going from (0,0) to (1,100) to (2,0) and is 0 for all inputs outside the range 0-2
	*/
	double ratioToScore(double ratio)
	{
		return (Math.max(0, Math.min(100*(1-Math.abs(1-ratio)), 100)));
	}
	
	/**
	 * Takes in a report on a target and compares the scores to the defined score limits to evaluate
	 * if the target is a hot target or not.
	 * 
	 * Returns True if the target is hot. False if it is not.
	 */
	boolean hotOrNot(TargetReport target)
	{
		boolean isHot = true;
		
		isHot &= target.tapeWidthScore >= TAPE_WIDTH_LIMIT;
		isHot &= target.verticalScore >= VERTICAL_SCORE_LIMIT;
		isHot &= (target.leftScore > LR_SCORE_LIMIT) | (target.rightScore > LR_SCORE_LIMIT);
		
		return isHot;
	}
}
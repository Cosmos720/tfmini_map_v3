void clustering(ArrayList<Points> modelPoints, ArrayList<Points> scenePoints){

  float[][] dataTransform = new float[modelPoints.size()*scenePoints.size()][4];
  for(int i=0; i<modelPoints.size(); i++){
    for(int j=0; j<scenePoints.size(); j++){
      dataTransform[i*scenePoints.size()+j] = transformCalcul(modelPoints.get(i), modelPoints.get((i+1)%modelPoints.size()), scenePoints.get(j), scenePoints.get((j+1)%scenePoints.size()));
      print("("+i+", "+j+")"+"[a] "+dataTransform[i*scenePoints.size()+j][0] + " [s] " + dataTransform[i*scenePoints.size()+j][1] + " [x] " + dataTransform[i*scenePoints.size()+j][2] + " [y] " + dataTransform[i*scenePoints.size()+j][3] + "\n");
    }
  }
}

float[] transformCalcul(Points modelA, Points modelB, Points sceneA, Points sceneB){
  float[] modelVector = modelA.vectorize(modelB);
  float[] sceneVector = sceneA.vectorize(sceneB);

  float theta = atan2(modelVector[1], modelVector[0]) - atan2(sceneVector[1],sceneVector[0]);

  float scale = modelA.distance(modelB)/sceneA.distance(sceneB);

  float deltaX = scale*sceneA.yCoord*sin(theta)-scale*sceneA.xCoord*cos(theta)+modelA.xCoord;
  float deltaY = -scale*sceneA.xCoord*sin(theta)-scale*sceneA.yCoord*cos(theta)+modelA.yCoord;

  float[] transform = {degrees(theta), scale, deltaX, deltaY};
  return transform;
}

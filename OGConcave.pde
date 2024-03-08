public class OGConcave {

    public OGConcave() {
    }

    ArrayList<Edge> computeConcaveHullFromConvexe(ArrayList<Points> pointsList, float n){
        //Preprocessing
        ArrayList<Points> points = (ArrayList<Points>)pointsList.clone();
        int[] hull = computeEnveloppe(points);
        ArrayList<Edge> convexeHull = new ArrayList<Edge>();
        for(int i = 0; i<hull.length; i++){
            convexeHull.add(new Edge(points.get(hull[i]), points.get(hull[(i+1)%hull.length])));
        }
        stroke(16, 100, 100, 10);
        for(int i=0; i<convexeHull.size(); i++){
            line(convexeHull.get(i).point1.xCoord, convexeHull.get(i).point1.yCoord, convexeHull.get(i).point2.xCoord, convexeHull.get(i).point2.yCoord);
        }
        ArrayList<Edge> concaveHull = (ArrayList<Edge>)convexeHull.clone();


        //Digging convex
        for(int i=0; i<concaveHull.size(); i++){
            Edge e = concaveHull.get(i);
            Points pk = nearestInnerPoint(e, concaveHull, points);
            if(pk==null){
                continue;
            }
            float eh = e.distance();
            float dd = min(pk.distance(e.point1), pk.distance(e.point2));
            if((eh/dd)>n){
                concaveHull.add(new Edge(e.point1, pk));
                concaveHull.add(new Edge(pk, e.point2));
                concaveHull.remove(e);
            }
        }
        return concaveHull;
    }

    int[] computeEnveloppe(ArrayList<Points> points){
        IntList enveloppe = new IntList();
        enveloppe.set(0, 0);
        enveloppe.set(1, 1);
        int pile = 1;
        for(int i=2; i<points.size(); i++){
            while((pile>=1) && prodVec3(points.get(enveloppe.get(pile-1)), points.get(enveloppe.get(pile)), points.get(i))>0){
                pile--;
            }
            pile++;
            enveloppe.set(pile, i);
        }
        for(int i=enveloppe.size()-1; i>pile; i--){
            enveloppe.remove(i);
        }
        return enveloppe.toArray();
    }

    float prodVec3(Points a, Points b, Points c){
        return (b.xCoord-a.xCoord)*(c.yCoord-a.yCoord)-(c.xCoord-a.xCoord)*(b.yCoord-a.yCoord);
    }

    Points nearestInnerPoint(Edge edge, ArrayList<Edge> concaveHull, ArrayList<Points> points){
        float distance = Float.MAX_VALUE;
        Points pk = null;
        Edge[] neighbors = neighborsEdge(edge, concaveHull);
        findPk:
        for(int i=1; i<points.size(); i++){
            Points p = points.get(i);
            if(distance<distEdge(edge, p)){
                continue;
            }
            if(distEdge(neighbors[0], p)<distEdge(edge, p) || distEdge(neighbors[1], p)<distEdge(edge, p)){
                continue;
            }
            for(Edge e: concaveHull){
                if(e.point1.equals(p) || e.point2.equals(p)){
                    continue findPk;
                }
            }
            distance = distEdge(edge, p);
            pk = p;
        }
        return pk;
    }

    Edge[] neighborsEdge(Edge edge, ArrayList<Edge> concaveHull){
        Edge[] neighbors = new Edge[2];
        for(int i=0; i<concaveHull.size(); i++){
            if(concaveHull.get(i).equals(edge)){
                continue;
            }
            if(concaveHull.get(i).point1.equals(edge.point1) || concaveHull.get(i).point2.equals(edge.point1)){
                neighbors[0] = concaveHull.get(i);
            }
            if(concaveHull.get(i).point1.equals(edge.point2) || concaveHull.get(i).point2.equals(edge.point2)){
                neighbors[1] = concaveHull.get(i);
            }
        }
        return neighbors;
    }

    float distEdge(Edge e, Points p){
        ////return min(p.distance(e.point1), p.distance(e.point2));
        float s = ((e.point1.yCoord-p.yCoord)*(e.point2.xCoord - e.point1.xCoord)-(e.point1.xCoord-p.xCoord)*(e.point2.yCoord - e.point1.yCoord))/pow(e.distance(), 2);
        float r = ((p.xCoord-e.point1.xCoord)*(e.point2.xCoord - e.point1.xCoord)+(p.yCoord-e.point1.yCoord)*(e.point2.yCoord - e.point1.yCoord))/pow(e.distance(), 2);
        ////return abs(s)*e.distance();
        if(r<=0){
            return p.distance(e.point1);
        }else if(r>=1){
            return p.distance(e.point2);
        }else{
            return abs(s)*e.distance();
        }
    }
}

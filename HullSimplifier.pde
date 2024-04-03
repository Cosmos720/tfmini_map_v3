ArrayList<Points> douglasPeucker(ArrayList<Points> array, float epsilon){
    float dmax = 0;
    int index = 0;
    for(int i=1; i<array.size()-1;i++){
        float d = distEdge(array.get(i), new Edge(array.get(0), array.get(array.size()-1)));
        if(d>dmax){
            index = i;
            dmax = d;
        }
    }

    if(dmax>epsilon){
        ArrayList<Points> res1 = douglasPeucker(new ArrayList<Points>(array.subList(0,index+1)), epsilon);
        ArrayList<Points> res2 = douglasPeucker(new ArrayList<Points>(array.subList(index,array.size())), epsilon);
        ArrayList<Points> res = new ArrayList<Points>(res1);
        res.addAll(res2);
        return res;
    }else{
        ArrayList<Points> res = new ArrayList<Points>();
        res.add(array.get(0));
        res.add(array.get(array.size()-1));
        return res;
    }

}

Object[] hullDivider(ArrayList<Points> array){
    Points p1 = array.get(0);
    Points p2 = null;
    float dmax = 0;
    for (Points p : array) {
        if(p1.distance(p) > dmax){
            dmax = p1.distance(p);
            p2 = p;
        }
    }
    if(p2 == null){
        return null;
    }
    int idx = array.indexOf(p2);
    return new Object[]{new ArrayList<Points>(array.subList(0, idx+1)), new ArrayList<Points>(array.subList(idx, array.size()))};
}
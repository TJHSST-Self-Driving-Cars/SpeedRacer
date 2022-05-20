struct Sequence{

  int sequenceBeginning;
  int sequenceEnd;
  int sequenceLength;
 
  void reset(){
  sequenceBeginning = 0;
  sequenceEnd = 0;
  sequenceLength = 0;
  }
 
  void set(Sequence s){
  sequenceBeginning = s.sequenceBeginning;
  sequenceEnd = s.sequenceEnd;
  sequenceLength = s.sequenceLength;
  }
 
};
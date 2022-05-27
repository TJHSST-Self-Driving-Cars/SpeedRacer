struct Sequence{

//these first two variables define the sequence and then the length is just extra information about the sequence that is pretty useful to have easy access to
int sequenceBeginning;
int sequenceEnd;
  int sequenceLength;
 
  //reset the Sequence so that it can be used for another iteration if needed
  void reset(){
  sequenceBeginning = 0;
  sequenceEnd = 0;
  sequenceLength = 0;
  }
 
  //set this sequence equal to another sequence
  void set(Sequence s){
  sequenceBeginning = s.sequenceBeginning;
  sequenceEnd = s.sequenceEnd;
  sequenceLength = s.sequenceLength;
  }
};

struct SequenceArrayList{

Sequence arrayList[360]; //store an array of Sequences which are initially all null. Then once you append a sequence, the array gets one more element of usable data
int currentSize = 0; //should always be the first null index in the array

  //get the size (pre simple)
  int getSize(){
    return currentSize;
  }
 
//add a sequence to the array
void appendSequence (Sequence element){
if (currentSize < 360){
arrayList[currentSize] = element;
currentSize++;
}
}

//get a sequence from a specified index
Sequence getSequence(int index){
if (index < currentSize)
return arrayList[index];
}

//reset the array so that it can be used for another iteration if needed
void reset(){
currentSize = 0;
}

 
 
};

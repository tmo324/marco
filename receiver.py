import rockBlock
from rockBlock import rockBlockProtocol

receivedData=""
receivedList=[]
 
class mtExample (rockBlockProtocol):
    
    def main(self):
        
        rb = rockBlock.rockBlock("/dev/ttyUSB0", self)
             
        rb.messageCheck()
                                                                  
        rb.close()
           
    def rockBlockRxStarted(self):
        print "rockBlockRxStarted"
        
    def rockBlockRxFailed(self):
        print "rockBlockRxFailed"
        
    def rockBlockRxReceived(self,mtmsn,data):
        print "rockBlockRxReceived " + str(mtmsn) + " " + data
        global receivedData
        receivedData=data
        global receivedList
        receivedList=receivedData.split(",")
        
    def rockBlockRxMessageQueue(self,count):
        print "rockBlockRxMessageQueue " + str(count)
             
#        
#if __name__ == '__main__':
#    mtExample().main()
#    print receivedData
#    print receivedList
#    print "JUST RECEIVED ABOVE. NOW DECODED"
#    print receivedList[1]



while True:
   mtExample().main()
   print receivedData
   print receivedList

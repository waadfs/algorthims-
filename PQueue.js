// Priority queue object constructor (document using JSDOC comments)
/**
 Create a PQ object
 @constructor
 @author SARA ALQAIDI
 @config {list} pq list
 @returns {PQueue}
 */
function PQueue()
{
    /**#@+
     * @description Property field - 
     */

    // user input fields

    /** head pointer of  priority queue
       @private */


    this.pq = new List(); // requirement: linked-list implementation

    /**#@- */
    // specify (design) methods
    /**#@+
     * @description <em>Member method - </em> 
     */
    /**  check priority queue is empty return true or  is not return return false [uses linklist package method: .isEmpty()]*/
    this.isEmpty = pqIsEmptyImpl;      // return true if queue empty

    /**remove/return item with minimum priority from the priority queue */
    this.deleteMin = pqDeleteMinImpl;     // remove/return item with minimum priority

    /** insert/update an item with priority */
    this.insert = pqInsertImpl;           // insert/update an item with priority

    /**#@- */

}

// -----------------------------------------------------------------------
// Priority queue node constructor (document using JSDOC comments)
/**
 used for defining vertex objects in priority queue
 @constructor
 @author SARA ALQAIDI
 @param {object} item input (configuration) object containing user field such as item and key
 @param {integer} key input
 */

function PQNode(item, key)
{
     /**#@+
     * @description Property field - 
     */

    // user input fields
    /** to store the item in the priority queue*/
    this.item = item;

    /**key to determine priority */
    this.prior = key;

    /**#@- */

    // specify (design) methods

}



// -----------------------------------------------------------------------

// functions used by PQueue() object methods
// specify interface information (JSDOC comments)

// function names should not clash with linklist.js and queue.js

// ....



//-----------------------------------------------------------------------
/**
check if the priority queue is empty or not
@methodOf PQueue#
@author SARA ALQAIDI
@returns {boolean} -  true if  priority queue is not empty otherwise false 
 */



function pqIsEmptyImpl() 

{   // return true if the queue is empty. 

    return this.pq.isEmpty(); 

} 



//-----------------------------------------------------------------------



/**
insert in priority queue or update the node's key 
@methodOf PQueue#
@author SARA ALQAIDI
@param {PQNode} item pq node
@param {integer} key represents the priority key
 */

function pqInsertImpl(item,key){

    //create new pq node 

    var item = new PQNode(item,key);

    var ptr = this.pq.first;

    var insert = true;//to check if its insert or update 

    if (!this.isEmpty()) { 

        while (ptr.next != null) { //Traverse in pq   

            if (item.item === ptr.item.item) { // we found same node in pq

               if(item.prior < ptr.item.prior){

                ptr.item.prior = item.prior;//update key 

                insert = false;// don't need to insert just update the key 

                break;//exit while   

                }

            }

            ptr = ptr.next;

        }

    }

    //----------------------------------------------

    if (insert) {//if insert true

        this.pq.insert(item); //using insert() from linklist 

    }

}

//-----------------------------------------------------------------------



/** 
delete the item with highest priority
@methodOf PQueue#
@author SARA ALQAIDI
@returns vertex with minimum weight 
 */

function pqDeleteMinImpl(){

    var min = this.pq.first;

    var ptr = this.pq.first;

    var prev = null;



    if (!this.isEmpty()) {

        while (ptr != null) {

            if (ptr.next != null && ptr.next.item.prior < min.item.prior) {

                min = ptr.next;

                prev = ptr;

            }

            ptr = ptr.next;

        }

        if (min != this.pq.first) { //if first element is not min

            prev.next = min.next;

        } else {

             this.pq.delete_first(); //if first element is the min, delete it using delete_first() from Linklist

        }

        return min.item;

    }

}

//-----------------------------------------------------------------------


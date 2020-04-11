const functions = require('firebase-functions');
const admin = require('firebase-admin');

/*
  Adds/subtracts any new values from the hourly total
*/
exports.aggregateData = functions.database.ref('ToF/{timeStamp}')
    .onWrite((change, context) => {
        const newDataRef = change.after;
        const beforeSnapshot = change.before;
        var timestamp = context.params.timeStamp;
        console.log("timeStamp: " +timestamp);
        var date = new Date(timestamp*1);
        var currentDate = date.getDate() + "-" + date.getMonth() + "-" + date.getFullYear();
        var currentHour = date.getHours();
        console.log("date: "+date);
        console.log("currentDate: " +currentDate);
        console.log("currentHour: " +currentHour);
        console.log("newDataRef:"+newDataRef);
        /* example: /Output/20-3-19/20/{{occupancy}} */
        var occupancyNotExists=false;
        var occupancy;
        return newDataRef.ref.parent.parent.child('Output').child(currentDate).child(currentHour).once('value').then(function(dataSnapshot){
            occupancyNotExists = (dataSnapshot.val() === null);

            console.log("occupancy: "+dataSnapshot.val());
            console.log("occupancyNotExists " +occupancyNotExists);
            
            occupancy = dataSnapshot.child("occupancy").val();
            enter = dataSnapshot.child("enter").val();
            exit = dataSnapshot.child("exit").val();
            var ref = newDataRef.ref.parent.parent.child('Output').child(currentDate).child(currentHour);
            if (occupancyNotExists){
                var newEnter = 0;
                var newExit = 0;
                var prevHourAgo = new Date(date - (1000*60*60));
                var newOccu = 0;
                console.log("prevHourAgoTimeStamp:" +prevHourAgo);
                return ref.parent.child(prevHourAgo.getHours()).once('value').then(function(prevHourSnap){
                    newOccu = prevHourSnap.child("occupancy").val();
                    console.log("prevHourSnap.val():"+newOccu);
                    if (newOccu ===null){
                        newOccu = 0;
                    }
                    
                    newDataRef.forEach(function(childSnap){
                        var isEnteredRoom = childSnap.child('EnterRoom').val();
                        if(isEnteredRoom){
                            newOccu +=1;
                            newEnter +=1;
                        }else{
                            newOccu -=1;
                            newExit +=1;
                        }
                    });
                    console.log("newOccu: "+newOccu);
                    ref.child("enter").set(newEnter);
                    ref.child("exit").set(newExit);
                    return ref.child("occupancy").set(newOccu);

                });

            }else{
                var childOccuBefore = 0;
                var childEnterBefore = 0;
                var childExitBefore = 0;
                beforeSnapshot.forEach(function(beforeChildSnap){
                    var isEnteredRoom = beforeChildSnap.child('EnterRoom').val();
                    if(isEnteredRoom){
                        childEnterBefore ++;
                        childOccuBefore +=1;
                    }else{
                        childExitBefore ++;
                        childOccuBefore -=1;
                    }
                });
                console.log("childOCcuBefore:" +childOccuBefore);
                var childOccuAfter = 0;
                var childEnterAfter = 0;
                var childExitAfter = 0;
                newDataRef.forEach(function(childSnap){
                    var isEnteredRoom = childSnap.child('EnterRoom').val();
                    console.log("isEnteredRoom: "+isEnteredRoom);
                    if(isEnteredRoom) {
                        childEnterAfter++;
                        childOccuAfter += 1;
                    } else {
                        childExitAfter++;
                        childOccuAfter -= 1;
                    }

                });
                console.log("childoccuAfter: "+childOccuBefore);
                occupancy += childOccuAfter - childOccuBefore;
                enter += childEnterAfter - childEnterBefore;
                exit += childExitAfter - childExitBefore;
                console.log("New occupancy: "+occupancy);
                ref.child("enter").set(enter);
                ref.child("exit").set(exit);
                return ref.child("occupancy").set(occupancy);
            }
        });

    }
);

exports.computeDaily = functions.database.ref('Output/{day}')
    .onWrite((change, context) => {
        const afterSnapshot = change.after;
        const beforeSnapshot = change.before;
        var day = context.params.day;

        console.log("context.params.day: "+day);
        var enterDay = 0;
        var exitDay = 0;
        var sumDay = 0;
        var count = 0;
        afterSnapshot.forEach(function(hourSnap){
            console.log("hourSnap.key"+hourSnap.key)
            if(!(hourSnap.key === "dailyOccupancy" || hourSnap.key === "dailyEnter" || hourSnap.key === "dailyExit" )){
                var enterH = hourSnap.child("enter").val();
                var exitH = hourSnap.child("exit").val();
                var occuH = hourSnap.child("occupancy").val();
                sumDay += occuH;
                enterDay += enterH;
                exitDay += exitH;
                count += 1;
            
                console.log("HourSnap, (enterH, exitH): "+hourSnap.key+", ("+enterH+", "+exitH+")");
                console.log("sumDay: " +sumDay);
            }
        });
        var dailyOccu = sumDay/count;
        console.log("dailyOccu: "+dailyOccu );
        afterSnapshot.ref.child("dailyEnter").set(enterDay);
        afterSnapshot.ref.child("dailyExit").set(exitDay);
        var currentTime = Date.now();
        console.log("CurrentTime: "+currentTime)
        afterSnapshot.ref.parent.parent.child("last_modified").set(currentTime);
        return afterSnapshot.ref.child("dailyOccupancy").set(dailyOccu);
    });

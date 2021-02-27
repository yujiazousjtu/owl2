#include "dealrobot.h"
#include "globaldata.h"
#include "staticparams.h"
#include "parammanager.h"
#include "matrix2d.h"
#include "field.h"
#include <iostream>
#include <qdebug.h>
namespace {
auto zpm = ZSS::ZParamManager::instance();
int DIFF_VECHILE_MAX_DIFF = 600;//1000
float MAX_SPEED = 5000;
// 由于图像的一次事故，帧率和系统延时改变了
const double TOTAL_MOV_LATED_FRAME = 4.0f; //平移的延时(原来为4.2)
const int NUM_MOV_LATENCY_FRAMES = static_cast<int>(TOTAL_MOV_LATED_FRAME);
const float MOV_LATENCY_FRACTION  = TOTAL_MOV_LATED_FRAME - static_cast<float>(NUM_MOV_LATENCY_FRAMES);

const double TOTAL_DIR_LATED_FRAME = 2.0f; //转动的延时
const int NUM_DIR_LATENCY_FRAMES = static_cast<int>(TOTAL_DIR_LATED_FRAME);
const float DIR_LATENCY_FRACTION  = TOTAL_DIR_LATED_FRAME - static_cast<float>(NUM_DIR_LATENCY_FRAMES);

const double Tolerant_Max_Error = 0.1;
}

CDealRobot::CDealRobot() {
    zpm->loadParam(fieldWidth, "field/canvasWidth", 13200);
    zpm->loadParam(fieldHeight, "field/canvasHeight", 9900);
    zpm->loadParam(minBelieveFrame, "AlertFusion/carMinBelieveFrame", 3);
    zpm->loadParam(theirMaxLostFrame, "AlertFusion/theirCarMaxLostFrame", 15);
    zpm->loadParam(ourMaxLostFrame, "AlertFusion/ourCarMaxLostFrame", 150);
    upPossible = 0.1;
    decidePossible = minBelieveFrame * upPossible;
    ourDownPossible = (1.0 - decidePossible) / ourMaxLostFrame;
    theirDownPossible = (1.0 - decidePossible) / theirMaxLostFrame;
    initPosCov = 100; posMeasErr = 20; posModelErr = 0.2; 
    timeStamp = 0;  
    for (int id = 0; id < PARAM::ROBOTMAXID; id++) {
        	posFilter[PARAM::BLUE][id] = PosFilter(initPosCov, posMeasErr, posModelErr);
        	posFilter[PARAM::YELLOW][id] = PosFilter(initPosCov, posMeasErr, posModelErr);
            //dirFilter[PARAM::BLUE][id] = PosFilter(initPosCov, posMeasErr, posModelErr);
            //dirFilter[PARAM::YELLOW][id] = PosFilter(initPosCov, posMeasErr, posModelErr);
            dirFilter[PARAM::BLUE][id] = DirFilter(initPosCov, posMeasErr, posModelErr);
            dirFilter[PARAM::YELLOW][id] = DirFilter(initPosCov, posMeasErr, posModelErr);
    }
    result.init(); 
    GlobalData::instance()->maintain.push(result);
}

double CDealRobot::posDist(CGeoPoint pos1, CGeoPoint pos2) {
    return std::sqrt((pos1.x() - pos2.x()) * (pos1.x() - pos2.x()) + (pos1.y() - pos2.y()) * ((pos1.y() - pos2.y())));
}

bool CDealRobot::isOnField(CGeoPoint p) {
    if (p.x() < fieldWidth / 2 && p.x() > -fieldWidth / 2 && p.y() < fieldHeight / 2 && p.y() > -fieldHeight / 2)
        return true;
    else
        return false;
}

bool CDealRobot::hasError(double data,double result){
    if(abs(result - data) / data >= Tolerant_Max_Error) return true;
    return false;
}

double CDealRobot::calculateWeight(int camID, CGeoPoint roboPos) {
    SingleCamera camera = GlobalData::instance()->cameraMatrix[camID];
    if (roboPos.x() > camera.leftedge.max && roboPos.x() < camera.rightedge.max &&
            roboPos.y() > camera.downedge.max && roboPos.y() < camera.upedge.max)
        return 1;
    else if (roboPos.x() < camera.leftedge.max && roboPos.x() > camera.leftedge.min)
        return abs(roboPos.x() - camera.leftedge.min) / abs(camera.leftedge.max - camera.leftedge.min);
    else if (roboPos.x() > camera.rightedge.max && roboPos.x() < camera.rightedge.min)
        return abs(roboPos.x() - camera.rightedge.min) / abs(camera.rightedge.max - camera.rightedge.min);
    else if (roboPos.y() < camera.downedge.max && roboPos.y() > camera.downedge.min)
        return abs(roboPos.y() - camera.downedge.min) / abs(camera.downedge.max - camera.downedge.min);
    else if (roboPos.y() > camera.upedge.max && roboPos.y() < camera.upedge.min)
        return abs(roboPos.y() - camera.upedge.min) / abs(camera.upedge.max - camera.upedge.min);
    else return 0.0001;//to deal with can see out of border situation
}

void CDealRobot::init() {
    zpm->loadParam(filteDir, "Vision/FilteDirection", false);
//    result.init();   

    for (int roboId = 0; roboId < PARAM::ROBOTMAXID; roboId++)
        for (int camId = 0; camId < PARAM::CAMERA; camId++) {
            //robotSeqence[PARAM::BLUE][roboId][camId].fill(-1, -32767, -32767, 0);
            //robotSeqence[PARAM::YELLOW][roboId][camId].fill(-1, -32767, -32767, 0);
            //robotSeqence[PARAM::BLUE][roboId][camId].fill(-1, -32767, -32767, 0, -32767, -32767);
            //robotSeqence[PARAM::YELLOW][roboId][camId].fill(-1, -32767, -32767, 0, -32767, -32767);
            robotSeqence[PARAM::BLUE][roboId][camId].fill(-1, -32767, -32767, 0);
            robotSeqence[PARAM::YELLOW][roboId][camId].fill(-1, -32767, -32767, 0);
        }         
    for (int i = 0; i < PARAM::CAMERA; i++) {
        if(GlobalData::instance()->cameraUpdate[i]) {
            for (int j = 0; j < GlobalData::instance()->camera[i][0].robotSize[PARAM::BLUE]; j++) {
                Robot robot = GlobalData::instance()->camera[i][0].robot[PARAM::BLUE][j];
                if ( GlobalData::instance()->robotPossible[PARAM::BLUE][robot.id] < decidePossible)
                    //当这是新车的时候//
                    robotSeqence[PARAM::BLUE][robot.id][i] = robot;
                else if  (lastRobot[PARAM::BLUE][robot.id].pos.dist(robot.pos) < DIFF_VECHILE_MAX_DIFF)
                    robotSeqence[PARAM::BLUE][robot.id][i] = robot;
            }
            for (int j = 0; j < GlobalData::instance()->camera[i][0].robotSize[PARAM::YELLOW]; j++) {
                Robot robot = GlobalData::instance()->camera[i][0].robot[PARAM::YELLOW][j];
                if ( GlobalData::instance()->robotPossible[PARAM::YELLOW][robot.id] < decidePossible)
                    robotSeqence[PARAM::YELLOW][robot.id][i] = robot;
                else if  (lastRobot[PARAM::YELLOW][robot.id].pos.dist(robot.pos) < DIFF_VECHILE_MAX_DIFF)
                    robotSeqence[PARAM::YELLOW][robot.id][i] = robot;
            }
        }
    }
    for (int i = 0; i < PARAM::ROBOTMAXID - 1; i++) {
        Robot temp(-32767, -32767, 0, -1);
        sortTemp[PARAM::BLUE][i] = temp;
        sortTemp[PARAM::YELLOW][i] = temp;
    }
}

void CDealRobot::mergeRobot() {
    for (int roboId = 0; roboId < PARAM::ROBOTMAXID; roboId++) {
        bool foundBlue = false, foundYellow = false;
        double blueWeight = 0, yellowWeight = 0;
        CGeoPoint blueAverage(0, 0), yellowAverage(0, 0);
        //CGeoPoint bluePixelAverage(0, 0), yellowPixelAverage(0, 0);
        RawInfo br, yr;
        double blueAngle = 0, yellowAngle = 0;
        for (int camId = 0; camId < PARAM::CAMERA; camId++) {
            double _weight = 0;
            if(robotSeqence[PARAM::BLUE][roboId][camId].pos.x() > -30000 && robotSeqence[PARAM::BLUE][roboId][camId].pos.y() > -30000) {
                foundBlue = true;
                _weight = calculateWeight(camId, robotSeqence[PARAM::BLUE][roboId][camId].pos);
                blueWeight += _weight;
                blueAverage.setX(blueAverage.x() + robotSeqence[PARAM::BLUE][roboId][camId].pos.x() * _weight);
                blueAverage.setY(blueAverage.y() + robotSeqence[PARAM::BLUE][roboId][camId].pos.y() * _weight);
                //bluePixelAverage.setX(bluePixelAverage.x() + robotSeqence[PARAM::BLUE][roboId][camId].rawPos.x() * _weight);
                //bluePixelAverage.setY(bluePixelAverage.y() + robotSeqence[PARAM::BLUE][roboId][camId].rawPos.y() * _weight);
                br.fill(robotSeqence[PARAM::BLUE][roboId][camId].raw);
                blueAngle = robotSeqence[PARAM::BLUE][roboId][camId].angle;
                //blueAngle += robotSeqence[PARAM::BLUE][roboId][camId].angle * _weight;
            }
            _weight = 0;
            if(robotSeqence[PARAM::YELLOW][roboId][camId].pos.x() > -30000 && robotSeqence[PARAM::YELLOW][roboId][camId].pos.y() > -30000) {
                foundYellow = true;
                _weight = calculateWeight(camId, robotSeqence[PARAM::YELLOW][roboId][camId].pos);
                yellowWeight += _weight;
                yellowAverage.setX(yellowAverage.x() + robotSeqence[PARAM::YELLOW][roboId][camId].pos.x() * _weight);
                yellowAverage.setY(yellowAverage.y() + robotSeqence[PARAM::YELLOW][roboId][camId].pos.y() * _weight);
                //yellowPixelAverage.setX(yellowPixelAverage.x() + robotSeqence[PARAM::YELLOW][roboId][camId].rawPos.x() * _weight);
                //yellowPixelAverage.setY(yellowPixelAverage.y() + robotSeqence[PARAM::YELLOW][roboId][camId].rawPos.y() * _weight);
                yr.fill(robotSeqence[PARAM::YELLOW][roboId][camId].raw);
                yellowAngle = robotSeqence[PARAM::YELLOW][roboId][camId].angle;
                //yellowAngle += robotSeqence[PARAM::YELLOW][roboId][camId].angle * _weight;
            }
        }
        if (foundBlue) {
            //Robot ave(blueAverage.x() / blueWeight, blueAverage.y() / blueWeight, blueAngle, roboId);
            //Robot ave(blueAverage.x() / blueWeight, blueAverage.y() / blueWeight, blueAngle, roboId, bluePixelAverage.x() / blueWeight, bluePixelAverage.y() / blueWeight);
            Robot ave(blueAverage.x() / blueWeight, blueAverage.y() / blueWeight, blueAngle, roboId, br);
            result.addRobot(PARAM::BLUE, ave);
        }
        if (foundYellow) {
            //Robot ave(yellowAverage.x() / yellowWeight, yellowAverage.y() / yellowWeight, yellowAngle, roboId);
            //Robot ave(yellowAverage.x() / yellowWeight, yellowAverage.y() / yellowWeight, yellowAngle, roboId, yellowPixelAverage.x() / yellowWeight, yellowPixelAverage.y() / yellowWeight);
            Robot ave(yellowAverage.x() / yellowWeight, yellowAverage.y() / yellowWeight, yellowAngle, roboId, yr);
            result.addRobot(PARAM::YELLOW, ave);
        }
    }
    if (PARAM::DEBUG) std::cout << "have found " << result.robotSize[PARAM::BLUE] << "blue car.\t" << result.robotSize[PARAM::YELLOW] << std::endl;
}

void CDealRobot::sortRobot(int color) {
    for (int id = 0; id < PARAM::ROBOTMAXID; id++) {
        bool found = false;
        for (int i = 0; i < result.robotSize[color]; i++)
            if (result.robot[color][i].id == id ) {
                if ((isOnField(result.robot[color][i].pos)) &&
                        (GlobalData::instance()->robotPossible[color][id] < decidePossible
                         || result.robot[color][i].pos.dist(lastRobot[color][id].pos) < DIFF_VECHILE_MAX_DIFF)) {
                    //result.robot[color][i].velocity = lastRobot[color][id].velocity;
                    //result.robot[color][i].accelerate = lastRobot[color][id].accelerate;
                    //result.robot[color][i].rotateVel = lastRobot[color][id].rotateVel;
                    lastRobot[color][id] = result.robot[color][i];
                    found = true;
                }
            }
        lastRobot[color][id].velocity = GlobalData::instance()->maintain[0].robot[color][id].velocity;
        lastRobot[color][id].accelerate = GlobalData::instance()->maintain[0].robot[color][id].accelerate;
        lastRobot[color][id].rotateVel = GlobalData::instance()->maintain[0].robot[color][id].rotateVel;
        
        if (found)  GlobalData::instance()->robotPossible[color][id] += upPossible;
        else { // 没看到车，猜测一个原始位置//
        	lastRobot[color][id].setPos(GlobalData::instance()->maintain[0].robot[color][id].pos + lastRobot[color][id].velocity / ZSS::Athena::FRAME_RATE);
        	lastRobot[color][id].angle = GlobalData::instance()->maintain[0].robot[color][id].angle + lastRobot[color][id].rotateVel / ZSS::Athena::FRAME_RATE;
            //lastRobot[color][id].setPos(lastRobot[color][id].pos + lastRobot[color][id].velocity / ZSS::Athena::FRAME_RATE);
            //lastRobot[color][id].angle = lastRobot[color][id].angle + lastRobot[color][id].rotateVel / ZSS::Athena::FRAME_RATE;
            if (GlobalData::instance()->robotPossible[color][id] >= decidePossible)
                if (GlobalData::instance()->commandMissingFrame[color] >= 20)
                    GlobalData::instance()->robotPossible[color][id] -= theirDownPossible;
                else
                    GlobalData::instance()->robotPossible[color][id] -= ourDownPossible;
            else GlobalData::instance()->robotPossible[color][id] -= decidePossible;
        }
//        if(color==0&&id==0) qDebug()<<"isFound"<<found;
        if (GlobalData::instance()->robotPossible[color][id] > 1.0) GlobalData::instance()->robotPossible[color][id] = 1.0;
        if(GlobalData::instance()->robotPossible[color][id] < 0.0)  GlobalData::instance()->robotPossible[color][id] = 0.0;
    }
    validNum[color] = 0;
    for (int id = 0; id < PARAM::ROBOTMAXID; id++) {
        if(GlobalData::instance()->robotPossible[color][id] >= decidePossible) sortTemp[color][validNum[color]++] = lastRobot[color][id];
    }
    //sort
    for (int i = 0; i < validNum[color] - 1; i++) {
        int maxj = i;
        for (int j = i + 1; j < validNum[color]; j++)
            if (GlobalData::instance()->robotPossible[color][sortTemp[color][maxj].id] <
                    GlobalData::instance()->robotPossible[color][sortTemp[color][j].id]) maxj = j;
        if (maxj != i) {
            Robot temp;
            temp = sortTemp[color][maxj];
            sortTemp[color][maxj] = sortTemp[color][i];
            sortTemp[color][i] = temp;
        }
    }
}

CGeoPoint CDealRobot::adjustPos(int team, int id, CGeoPoint point, double angle) {
    if (abs(GlobalData::instance()->robotCommand[team][0].robotSpeed[id].vx) > 0.001
                || abs(GlobalData::instance()->robotCommand[team][0].robotSpeed[id].vy) > 0.001) {
            //小数部分
            auto command = GlobalData::instance()->robotCommand[team][0 - NUM_MOV_LATENCY_FRAMES].robotSpeed[id];
            CVector robot_travel = CVector(command.vx, -command.vy) * MOV_LATENCY_FRACTION / double(ZSS::Athena::FRAME_RATE);

            //整数部分
            for (int frame = NUM_MOV_LATENCY_FRAMES - 1; frame >= 0; frame--) {
                command = GlobalData::instance()->robotCommand[team][0 - frame].robotSpeed[id];
                robot_travel = robot_travel + CVector(command.vx * 10, -command.vy * 10) / ZSS::Athena::FRAME_RATE;
            }

            robot_travel = robot_travel.rotate(angle);
            return (point + robot_travel);
    }
    else return point;
}

void CDealRobot::updateVel(int team, ReceiveVisionMessage& result) {
    if(team == 0) timeStamp += 1 / ZSS::Athena::FRAME_RATE;
    for (int i = 0; i < validNum[team]; i++) {
        Robot & robot = result.robot[team][i];
//        bool side;
        //朝向滤波
        auto &playerRotVel = dirFilter[team][robot.id];
        if(!playerRotVel.isInit()) {
            playerRotVel.initState(robot.angle, robot.rotateVel);
        }
        double Rotdt = timeStamp - playerRotVel.lastTimestamp;
        playerRotVel.lastTimestamp = timeStamp;
        playerRotVel.setTransitionMat(Rotdt);
        //playerPosVel.predict2(lastRobot[team][robot.id].accelerate);
        playerRotVel.predict();
        playerRotVel.update2(robot.angle);

        double filterDir = playerRotVel.postEstimatedDir();
        double rotVel = playerRotVel.postEstimatedVel();

        robot.angle = filterDir;
        //robot.angle = playerRotVel.normalize(robot.angle);
        //playerRotVel.set(0, robot.angle);
        robot.rotateVel = rotVel;
        /**auto &playerRotVel = dirFilter[team][robot.id].update(std::cos(robot.angle), std::sin(robot.angle));
        double filterDir = std::atan2(playerRotVel(1, 0), playerRotVel(0, 0));
        double rotVel = playerRotVel(2, 0) * std::cos(90 * 3.1416 / 180 + filterDir)
                        + playerRotVel(3, 0) * std::sin(90 * 3.1416 / 180 + filterDir);

        if (abs(GlobalData::instance()->robotCommand[team][0].robotSpeed[robot.id].vr) > 0.001) {
            //小数部分
            auto command = GlobalData::instance()->robotCommand[team][0 - NUM_MOV_LATENCY_FRAMES].robotSpeed[robot.id];
            robot.angle += - command.vr * DIR_LATENCY_FRACTION / ZSS::Athena::FRAME_RATE;
            //整数部分
            for (int frame = NUM_DIR_LATENCY_FRAMES - 1; frame >= 0 ; frame--) {
                command = GlobalData::instance()->robotCommand[team][0 - frame].robotSpeed[robot.id];
                robot.angle += - command.vr  / ZSS::Athena::FRAME_RATE;
            }
            robot.rotateVel = - command.vr;
            //qDebug()<<"robot.angle"<<robot.angle;
        }
        else {
            robot.angle = filterDir;
            //qDebug()<<"angle"<<robot.angle;
            robot.rotateVel = rotVel;
            //robot.rotateVel = rotVel * ZSS::Athena::FRAME_RATE;
        }**/

        //位置滤波
        auto & playerPosVel = posFilter[team][robot.id];
        if(!playerPosVel.isInit()) {
            playerPosVel.initState(robot.pos, CVector(robot.velocity));
        }
        //CGeoPoint filtPoint = playerPosVel.postEstimatedPos();
        //qDebug()<<"filtPoint"<<filtPoint.x()<<filtPoint.y();
        double Posdt = timeStamp - playerPosVel.lastTimestamp;
        playerPosVel.lastTimestamp = timeStamp;
//        if(team==0&&robot.id==0){
//            qDebug()<<"team"<<team<<"id"<<robot.id<<"dt"<<dt;
//        }
        playerPosVel.setTransitionMat(Posdt);
        //playerPosVel.predict2(lastRobot[team][robot.id].accelerate);
        playerPosVel.predict();
        playerPosVel.update2(robot.pos);
        
        /**if(hasError(robot.pos.x(), playerPosVel.postEstimatedPos().x())) {
            qDebug()<<"Error_x";
            playerPosVel.set(0, adjustPos(team, robot.id, robot.pos, robot.angle).x());
            //playerPosVel.set(0, robot.pos.x());
            //playerPosVel.set(0, robot.pos.x() + GlobalData::instance()->robotCommand[team][0].robotSpeed[robot.id].vx / ZSS::Athena::FRAME_RATE);
            //playerPosVel.set(0, playerPosVel.postEstimatedPos().x() + GlobalData::instance()->robotCommand[team][0].robotSpeed[robot.id].vx * dt);
            playerPosVel.set(2, GlobalData::instance()->robotCommand[team][0].robotSpeed[robot.id].vx);
        }
        if(hasError(robot.pos.y(), playerPosVel.postEstimatedPos().y())) {
            qDebug()<<"Error_y";
            playerPosVel.set(1, adjustPos(team, robot.id, robot.pos, robot.angle).y());
            //playerPosVel.set(1, robot.pos.y());
            //playerPosVel.set(1, robot.pos.y() - GlobalData::instance()->robotCommand[team][0].robotSpeed[robot.id].vy / ZSS::Athena::FRAME_RATE);
            //playerPosVel.set(0, playerPosVel.postEstimatedPos().y() - GlobalData::instance()->robotCommand[team][0].robotSpeed[robot.id].vy * dt);
            playerPosVel.set(3, -GlobalData::instance()->robotCommand[team][0].robotSpeed[robot.id].vy);
        }**/

        CGeoPoint filtPoint = playerPosVel.postEstimatedPos();
        CVector PlayVel = playerPosVel.postEstimatedVel();

        //ZSS::ZParamManager::instance()->loadParam(side, "ZAlert/IsRight");
        robot.pos = filtPoint;
        robot.velocity = PlayVel;
        robot.velocity = robot.velocity.rotate(robot.angle);
        playerPosVel.set(2, robot.velocity.x());
        playerPosVel.set(3, robot.velocity.y());
        //robot.velocity = PlayVel * ZSS::Athena::FRAME_RATE;
        robot.accelerate = (robot.velocity - lastRobot[team][robot.id].velocity) / Posdt;
        //robot.accelerate = robot.accelerate.rotate(robot.angle);
        //robot.accelerate = (robot.velocity - GlobalData::instance()->maintain[-2].robot[team][robot.id].velocity) / 3 * ZSS::Athena::FRAME_RATE ;
        //robot.accelerate = (robot.velocity - GlobalData::instance()->maintain[0].robot[team][robot.id].velocity) * ZSS::Athena::FRAME_RATE ;
        if(robot.accelerate.mod()>7000) robot.accelerate=CVector(std::cos(robot.angle), std::sin(robot.angle))*7000;
        else if(robot.accelerate.mod()<-7000) robot.accelerate=CVector(std::cos(robot.angle), std::sin(robot.angle))*(-7000); 

        /**if(team==0&&robot.id==0){
            qDebug()<<"team"<<team<<"id"<<robot.id<<"pos"<<robot.pos.x()<<robot.pos.y();
            qDebug()<<"vel"<<robot.velocity.x()<<robot.velocity.y();
            //qDebug()<<"acc"<<robot.accelerate.x()<<robot.accelerate.y();
            //qDebug()<<"maintain"<<GlobalData::instance()->maintain[0].robot[team][robot.id].velocity.x()<<GlobalData::instance()->maintain[0].robot[team][robot.id].velocity.y();
            qDebug()<<"command"<<GlobalData::instance()->robotCommand[team][0].robotSpeed[robot.id].vx<<GlobalData::instance()->robotCommand[team][0].robotSpeed[robot.id].vy;
        }**/

        lastRobot[team][robot.id].velocity = robot.velocity;
        lastRobot[team][robot.id].accelerate = robot.accelerate;
        lastRobot[team][robot.id].rotateVel = robot.rotateVel;
    }
}

void CDealRobot::run() {
    init();
    mergeRobot();
    sortRobot(PARAM::BLUE);
    sortRobot(PARAM::YELLOW);
    result.init();
    //重新加入概率排序后的车
    for (int i = 0; i < validNum[PARAM::BLUE]; i++)
        result.addRobot(PARAM::BLUE, sortTemp[PARAM::BLUE][i]);
    for (int i = 0; i < validNum[PARAM::YELLOW]; i++)
        result.addRobot(PARAM::YELLOW, sortTemp[PARAM::YELLOW][i]);
    //qDebug("try to push robot");
    GlobalData::instance()->processRobot.push(result);
}



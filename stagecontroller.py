#universal motor controllers demo
#사용 중

import socket as sk
# import threading
import math
import time

# 빠른 출력
import sys
fastprint = sys.stdout.write

# input = sys.stdin.readline

# 글로벌 명령어 설정
from enum import Enum, auto

# 시스템 로그 관련 모듈
import logging
from pythonjsonlogger import jsonlogger

from PySide6.QtCore import QObject, QThread, QTimer, QMutex, QMutexLocker, Signal, Slot, QEventLoop
from PySide6.QtTest import QTest # 동작 멈추기

# 딕셔너리 꼴의 명령체계를 위한 모듈들
from dataclasses import dataclass, field, asdict
from typing import Optional, Union, Dict, Any

from queue import Queue

# -------------------------------------------------------------------------------------
# 스테이지 정보 부여를 위한 클래스
class Stage:
    def __init__(self, model = str, old = False, axisdir = True, simulation = False):
        
        # 컨트롤러 연결 정보 (연결 컨트롤러 번호 및 포트)
        self.ctrler_num = None
        self.ctrler_port = None
        
        # 사용 안함 : 스테이지 직접 명령 구현용
        self.ctrler_socket_id = None    # socket id / None / simulation
        
        # 시뮬레이션 스테이지 여부
        # 아직 안함
        self.stage_simulation = simulation
        
        # 구성품 소속
        self.component = None
        
        # 현재 작업 내용
        self.component_process = None
        
        # 모터 스탭 정보
        self.steprev = 51200            # 고정값
        
        # 모델 정보
            # 스테이지 정보 생성
        self.model_name = model         # 모델명
        self.stage_name_info: list= None     # 모델명에서 얻는 정보 코드
        self.stage_type: str= None           # 스테이지 종류
        self.stage_tablesize: str= None      # 테이블 크기
        self.stage_tableinfo: str= None      # 테이블 정보
        self.stage_etcinfo: str= None        # 그 외 정보
        
        self.stage_old_version = old       # 스테이지 구형 버전: True 신형 : False
        
            # 모델명에서 변환된 정보 생성
        self.stage_unit = str           # 스테이지 단위
        
        self.zstage_slope = None        # z축 슬로프
        self.tstage_rotation_center = None  # 고니오미터 tilt축 회전중심
        self.rstage_wgratio = None      # 회전 스테이지 웜기어비
        self.stage_max_position: int= None   # 안전하게 움직일 수 있는 최대동작 값 (원점기준)
        self.stage_limits: list= None        # 동작 제한 범위
        self.stage_limitvalue = 0.1     # 범위 제한을 위한 여유값
        
        
            # 모델정보 변환
        self.StageInfo(model = self.model_name)
        
        
        # 자체 설정 정보
            # 상태 관련 정보
        self.stgae_updating = False           # 상태 최신화 동작 여부
        self.stage_orderable = True           # 명령 가능 상태
        
        self.motion = None              # 현재 동작 정보
        self.status = None              # 현재 상태 정보
        self.step: int= None            # 현재 스탭 정보
        self.position: float= None      # 현재 위치정보 (스탭으로부터 변환)
        
        
            # 속도 관련 정보
        self.speed = None               # 스테이지 이송 속도 (Lv1 = 1mm/s)
        self.speed_limit = 15              # 이송속도 제한
        
            # 감가속도 관련 정보 - 아직 없음
        self.acceleration = None        # 이송 가속도
        self.deceleration = None        # 이송 감속도
        self.accel_limit = None         # 감가속도 제한
        
            # 가상 축 정보
        self.stage_axis = axisdir          # True : 정방향 / False : 역방향
        
        
        
        # 스테이지 상태 변수 초기화
        
        # 0번 예비
        self.driver_error: bool= None         # 드라이버 에러
        self.power_off: bool= None          # 모터파워 off (1) /on(0)
        self.stop: bool= None                # 모터정지 (1) / 모터동작(0)
                                        
        self.homestop: bool= None            # 원점동작완료(1) / 동작 중(0)
        self.requested_position: bool= None  # 요청위치도달(1) / 도달 중(0)
        self.requested_speed: bool= None     # 요청속도도달(1) / 도달 중(0)
        # 7번 예비
        
        self.overheat_warning: bool= None    # 과열주의
        self.overheat_error: bool= None      # 과열에러
        # 10번 예비
        # 11번 예비
        
        self.cw_limit: bool= None            # CW limt 상태      (Active - 1)
        self.ccw_limit: bool= None           # CCW limit 상태    (Active - 1)
        self.homeswitch: bool= None          # HomeSwitch 상태   (Active - 1)
        # 15번 예비
        
        self.direction_reversed: bool= None  # 모터방향 반전    (Active - 1)
        self.sensor_reversed: bool= None     # 제한센서 반전    (Active - 1)
        # 18번 예비
        # 19번 예비
        
        # 이후에 자체적으로 상태 업데이트?
        
    # -------------------------------------------------------------------------------------
    def StageInfo(self, model = str):
        '''모델명으로부터 정보 변환'''
        self.model_name = model.upper()
        self.stage_name_info = list(map(str,self.model_name.split('-')))
        
        # AM1-0810-3S
        # AT2-0675-3S
        
        # 스테이지 타입, 단위 설정
        self.stage_type = self.stage_name_info[0][1]
        self.stage_tablesize = self.stage_name_info[1][:2]   # str -> int 변환
        self.stage_tableinfo = self.stage_name_info[1][2:]   # str -> int 변환
        self.stage_etcinfo = self.stage_name_info[2]
        
        # 상판 사이즈
        self.stage_tablesize = int(self.stage_tablesize + '0')
        
        linearstage = ['M', 'L', 'Z']
        nonlinearstage = ['R', 'T']
        
        if self.stage_type in linearstage:
            # 선형 스테이지 3종
            self.stage_unit = 'mm'
            self.stage_tableinfo = int(self.stage_tableinfo + '0') # 이송거리
            
            # 미니어처 스테이지
            if self.stage_type == 'M':
                pass
            # 선형 스테이지
            elif (self.stage_type == 'L') & (self.stage_tablesize == 40):
                # 40*40 사이즈 테이블만 해당
                self.stage_tableinfo = int(0.1 * self.stage_tableinfo)
            # z축 스테이지
            elif self.stage_type == 'Z':
                self.zstage_slope = 20
                self.stage_tableinfo = int(0.1 * self.stage_tableinfo)
            
            self.stage_max_position = (self.stage_tableinfo / 2) - self.stage_limitvalue
        
        elif self.stage_type in nonlinearstage:
            # 회전 스테이지 2종
            self.stage_unit = '°'
            
            # 회전 스테이지
            # 정보 어떻게 반환하는지 불명
            # 추후 업데이트 예정
            if self.stage_type == 'R':
                self.rstage_wgratio = 180
                
                if self.stage_tableinfo == '00':
                    self.stage_max_position = 360
                else:
                    self.stage_max_position = int(self.stage_tableinfo) * 0.5 - self.stage_limitvalue
            
            # 고니오미터
            elif self.stage_type == 'T':
                self.tstage_rotation_center = int(self.stage_tableinfo)
                
                if self.tstage_rotation_center == 50:
                    self.stage_max_position = 8 - self.stage_limitvalue
                elif self.tstage_rotation_center == 75:
                    self.stage_max_position = 5.5 - self.stage_limitvalue
                elif self.tstage_rotation_center == 10:
                    self.tstage_rotation_center = 100
                    self.stage_max_position = 5 - self.stage_limitvalue
                elif self.tstage_rotation_center == 125 or 122:
                    self.stage_max_position = 4 - self.stage_limitvalue
                    
        self.stage_limits = [-self.stage_max_position, self.stage_max_position]

    # -------------------------------------------------------------------------------------
    
    # 스탭 <-> 위치 변환
    
    def convert_step_to_position(self, step):
        '''스탭을 위치값으로 변환'''
        step = int(step)
        
        if self.stage_type == 'Z':
            pos = self.Step2Zposition(step)
        elif self.stage_type == 'T':
            pos = self.Step2TDegree(step)
        elif self.stage_type == 'R':
            pos = self.Step2RDegree(step)
        else:
            pos = step / self.steprev
        
        # 소수점 3자리에서 반올림
        pos = round(pos, 3)
        
        # 가상 축 변환
        if self.stage_axis == False:
            if pos == 0:
                pass
            else:
                pos = -pos
        
        self.position = pos
        
        return pos # 단위 정보 없는 실수형 변수
    
    def convert_position_to_step(self, pos):
        '위치값을 스탭으로 변환'
        pos = float(pos)
        
        if self.stage_type == 'Z':
            step = self.Zposition2Step(pos)
        elif self.stage_type == 'T':
            step = self.TDegree2Step(pos)
        elif self.stage_type == 'R':
            step = self.RDegress2Step(pos)
        else:
            step = self.steprev*pos
        
        # 정수로 반올림
        step = round(step)
        
        # 가상 축 변환
        if self.stage_axis == False:
            if step == 0:
                pass
            else:
                step = -step
        
        self.step = step
        
        return step
    
    # 변환식은 변환만 진행
        # Z-axis
    def Zposition2Step(self, zpos= float):
        '''z축모터 높이를 스탭으로 변환'''
        step = self.steprev * (zpos / math.tan(self.zstage_slope*(math.pi/180)))
        step = round(step)
        
        return step
    
    def Step2Zposition(self, step= int):
        '''스탭을 z축모터 높이로 변환'''
        zpos = (step / self.steprev)*math.tan(self.zstage_slope*(math.pi/180))
        zpos = round(zpos, 3)
        
        return zpos
 
        # Tilt
    def TDegree2Step(self, degree= float):
        '''고니오미터 각도를 스탭으로 변환'''
        rad = degree*(math.pi/180) # 각도 -> 라디안 변환
        step = (self.tstage_rotation_center * self.steprev) * math.sin(rad)
        
        return step
    
    def Step2TDegree(self, step= int):
        '''스탭을 고니오미터 각도로 변환'''
        rad = math.asin((step / (self.tstage_rotation_center * self.steprev)))
        pos = rad * (180 / math.pi) # 라디안 -> 각도 변환
        
        return pos
    
        # Rotary
    def RDegress2Step(self, degree = float):
        '''
        회전각을 스탭으로 변환
        '''
        rad = degree *(self.steprev / 2)
        step = round(rad)
        
        return step
    
    def Step2RDegree(self, step = int):
        '''
        스탭을 회전각으로 변환.
        '''
        rad = step*(2 / self.steprev)
        pos = round(rad,3)
        
        # Resolution : 0.004°
        # Accuracy : 0.05°
        # Backlash : 0.15°
        return pos

    # -------------------------------------------------------------------------------------
    # 스테이지 상태 정보 변환
    
    def statuscode_converter(self, statuscode=str):
        '''모터 상태코드로부터 정보를 나눔'''
        # 상태코드 저장
        self.status = statuscode
        
        # statuscode로부터 정보 가져오기
        # 0번 예비
        self.driver_error = bool(int(statuscode[1]))           # 드라이버 에러
        self.power_off = bool(int(statuscode[2]))           # 모터파워 off (1) /on(0)
        self.stop = bool(int(statuscode[3]))             # 모터정지 (1) / 모터동작(0)
                                        
        self.homestop = bool(int(statuscode[4]))         # 원점동작완료(1) / 동작 중(0)
        self.requested_position = bool(int(statuscode[5]))         # 요청위치도달(1) / 도달 중(0)
        self.requested_speed = bool(int(statuscode[6]))        # 요청속도도달(1) / 도달 중(0)
        # 7번 예비
        
        self.overheat_warning = bool(int(statuscode[8]))     # 과열주의
        self.overheat_error = bool(int(statuscode[9]))     # 과열에러
        # 10번 예비
        # 11번 예비
        
        self.cw_limit = bool(int(statuscode[12]))           # CW limt 상태      (Active - 1)
        self.ccw_limit = bool(int(statuscode[13]))          # CCW limit 상태    (Active - 1)
        self.homeswitch = bool(int(statuscode[14]))      # HomeSwitch 상태   (Active - 1)
        # 15번 예비
        
        self.direction_reversed = bool(int(statuscode[16]))    # 모터방향 반전    (Active - 1)
        self.sensor_reversed = bool(int(statuscode[17]))    # 제한센서 반전    (Active - 1)
        # 18번 예비
        # 19번 예비
        

        # 상태 코드로부터 정보 변환
            # 장비 구동 멈추고 몇초 후 명령 가능 상태로 바꾸기
            # 에러 발생 또는 과열되면 긴급 정지 후 파워 off
        
    # -------------------------------------------------------------------------------------
    
    # 스테이지 다이렉트 메세지 통신
    # 기능으로만 구현 되어있고, 거의 쓸 일 없을 것임
    
    def DMsend(self, msg):
        stgskt = self.ctrler_socket_id
        stgskt.send(msg.encode('unicode_escape'))
        time.sleep(0.0025)
        
    def DMsendrecv(self, msg):
        stgskt = self.ctrler_socket_id
        stgskt.send(msg.encode('unicode_escape'))
        # 구형 컨트롤러 메세지 반환값 오류 없는 최저값
        time.sleep(0.003)
        data = stgskt.recv(1024).decode('unicode_escape')
        
        return data
    
    # -------------------------------------------------------------------------------------
    
    # 스테이지 다이렉트 명령
    # 아직 사용 안함
    
    def CurrentPos(self):
        '''현재 위치'''
        stepmsg = str(self.DMsendrecv(self.ctrler_port + 'P'))[:-1] # 반환 데이터 원문
        stepidx = stepmsg.find('POS')
        step = stepmsg[stepidx+3:]
        pos = self.convert_step_to_position(step)
        
        self.position = pos
        
        return pos
    
    def CurrentStat(self):
        '''현재 상태'''
        statmsg = str(self.DMsendrecv(self.ctrler_port + 'STATUS'))[:-1] # 반환 데이터 원문
        statidx = statmsg.find('STATUS')
        stat = statmsg[statidx+6:].replace('_','')
        stat = ''.join(reversed(stat))
        self.statuscode_converter(stat)
        
        return stat
    
    # -------------------------------------------------------------------------------------
        # 스테이지 구동  
    def GoHome(self):
        '''위치 초기화(원점으로 이동)'''
        if self.stage_orderable:
            cmds = [self.ctrler_port + 'ST0',
                    self.ctrler_port + 'MA0',
                    self.ctrler_port + 'HM1']
            [self.DMsend(cmd) for cmd in cmds]
        
    def GoTarget(self, coord = 0, spdLv = 1):
        '''설정된 속도로 입력 위치 이동'''
        if self.stage_orderable:
            if (coord <= abs(self.stage_max_position)) and (spdLv <= self.speed_limit):
                
                speed = self.steprev*spdLv
                target = self.convert_position_to_step(coord)
                
                cmds = [f'{self.ctrler_port}MA1',
                        f'{self.ctrler_port}V' + str(speed),
                        f'{self.ctrler_port}D' + str(target),
                        f'{self.ctrler_port}G']
                
                # self.stage_orderable = False # 스테이지 정보 최신화에서 구현
                # DMsend로 하면 신형 컨트롤러 작동 하지 않음
                [self.DMsendrecv(cmd) for cmd in cmds]
                
            elif (coord) > abs(self.stage_max_position):
                fastprint(f'{self.ctrler_num} - {self.ctrler_port} : 가동 제한 범위({self.stage_limits})를 벗어났습니다.\n')
                
            elif (spdLv > self.speed_limit):
                fastprint(f'{self.ctrler_num} - {self.ctrler_port} : 최대 속도({self.speed_limit})를 초과했습니다.\n')
            else:
                fastprint('제한 범위를 벗어나거나 최대속도를 초과 했습니다.\n')
    
    
    def MoveDist(self, distance = 0, spdLv = 1):
        '''현재 위치에서 설정된 속도와 거리만큼 이동'''
        if self.stage_orderable:
            # self.position에 대한 구현 x
                # 현위치 1회 업데이트
                # 컨트롤러 클래스에서 스테이지 움직일 때마다 계속 위치 정보 최신화 해줄 예정
            
            if (self.position + distance <= abs(self.stage_max_position)) and (spdLv <= self.speed_limit):
                
                speed = self.steprev*spdLv
                target = self.convert_position_to_step(distance)
                
                cmds = [self.ctrler_port + 'MA0',
                        self.ctrler_port + 'V' + str(speed),
                        self.ctrler_port + 'D' + str(target),
                        self.ctrler_port + 'G']
                
                # self.stage_orderable = False # 스테이지 정보 최신화에서 구현
                [self.DMsendrecv(cmd) for cmd in cmds]
            elif (self.position + distance) > abs(self.stage_max_position):
                fastprint(f'{self.ctrler_num} - {self.ctrler_port} : 가동 제한 범위()를 벗어났습니다.\n')
            elif (spdLv > self.speed_limit):
                fastprint(f'{self.ctrler_num} - {self.ctrler_port} : 최대 속도()를 초과했습니다.\n')
        else:
            fastprint('제한 범위를 벗어나거나 최대속도를 초과 했습니다.\n')
        
    def Stop(self):
        '''스테이지 동작 정지'''
        self.DMsend(self.ctrler_port + 'S')
        
        # cmds = [self.ctrler_port + 'S',
        #         self.ctrler_port + 'ST1']
        
        # [self.DMsend(cmd) for cmd in cmds]
    
    # -------------------------------------------------------------------------------------
    
    # 작업예정 기능
    
    def MoveJog(self, direction, spdLv):
        '''지정한 방향으로 조그를 이동'''
        pass
    
# 스테이지 구동을 위한 클래스
class Controller(QObject):
    # 자동 스테이지 컨트롤러 송신 --------------------------------------------------------------------------------------------------------
    # 데이터매니저로 송신할 데이터
    signal_systemlog = Signal(object)   # 시스템 로그
    
    # 컨트롤러 상태 정보(명령가능여부, 업데이팅 여부, 초기화 횟수, 동작 횟수, 준비)
    signal_status = Signal(dict, dict, dict, dict, bool)
    
    # 컨트롤러 생성 데이터(연결목록, 상태, 스탭, 위치, 모션)
    signal_data = Signal(dict, dict, dict, dict, dict)
    
    # 자동 스테이지 컨트롤러 수신 --------------------------------------------------------------------------------------------------------
    # 데이터매니저에서 수신할 데이터
    # 데이터 요청
    @Slot(bool, bool)
    def request_process(self, operating, collecting):
        '''데이터 매니저로부터 데이터 처리 요청 수신'''
        with QMutexLocker(self.mutex):
            self.request_operating = operating
            self.request_collecting_info = collecting
        print(f'스테이지컨트롤러에서 동작 및 수집 요청 잘 받았소 : 동작{self.request_operating}, 수집{self.request_collecting_info}')
    
    # 구동 명령 추가
    @Slot(object) # 더 추가?
    def receive_command(self, cmd: object):
        '''데이터매니저로부터 명령 수신 및 명령 Queue에 추가'''
        # 모듈간 유기성 없애고 데이터매니저 중앙화
        with QMutexLocker(self.mutex):
            # 명령 번호 부여
            self.command_counter += 1
            
            # cmd['number'] = self.command_counter
            # print(self.command_counter)
            self.command_queue.put(cmd)

            # fastprint(f'수행 예정 명령어 개수:{self.command_queue.qsize()}\n')
    
    # 컨트롤러 켜고 끄기?
    # @Slot
    # def received_ctrler_onoff(self, num, power):
    #     pass
    
    # 생성자 ---------------------------------------------------------------------------------------------------------------------------
    def __init__(self):
        super().__init__()
        # 멀티스레드 환경에서의 안전한 데이터 읽쓰 보장
        self.mutex = QMutex()

        # 동작 및 데이터 요청 여부
        self.requested_operating = False
        self.requested_collecting_info = False

        # 이벤트 루프 내 명령 수행 여부
        self.excuting_command = False       # 명령수행 순서 보장

        # 명령 큐
        self.command_queue = Queue()        # 동작명령1 dict -> 동작명령2 dict -> ...
        self.command_counter = 0
        
        # 동작명령 받은 스테이지들 큐
        self.operation_queue = Queue()      # [컨트롤러 번호1, 컨트롤러 포트1] -> [컨트롤러 번호2, 컨트롤러 포트2] -> ...
        self.updating_controllers = set()   # {1, 2, ...} : 동작명령을 받아 업데이팅 할 컨트롤러 번호, 중복없는 내림차순 정렬 세트
                                            # 명령 수행 과정에서 sorted(set)에 의해 리스트로 바뀜
        
        # 컨트롤러 상태 정보 - 일부 데이터매니저로 보냄
        self.ctrler_sockets = {}        # {ctrler_0 : socket id  / simulation, ... }
        self.ctrler_ipadress = {}       # {ctrler_0 : '192.168.0.123' / simulation, ... }
        self.ctrler_orderable = {}      # {ctrler_0 : False, ... } - 명령 가능 상태
        self.ctrler_updating = {}       # {ctrler_0 : False, ... } - 현재 위치 업데이트 여부
        
        self.ctrler_old_ver = {}        # {ctrler_0 : False, ... } - 구형 / 신형 버전
        self.ctrler_simulated = {}      # {ctrler_0 : False, ... } - 더미 컨트롤러
        self.ctrler_simulating = {}     # {ctrler_0 : False, ... } - 스테이지 시뮬레이팅
        
        # 컨트롤러 포트 수(고정값)
        self.ctrler_ports = 4
        
        
        # 컨트롤러 생성 데이터 - 데이터매니저로 보냄
        self.stages = {}            # {ctrler_0 : [x_axis, y_axis, theta_x, theta_y], ... }

        self.statuses = {}          # {ctrler_0 : ['Bin code', 'Bin code', 'Bin code', 'Bin code'], ... }
        self.motions = {}           # {ctrler_0 : ['R', 'B', 'S', 'R'], ... }
        self.steps = {}             # {ctrler_0 : [0, 0, 0, 0], ... }
        self.positions = {}         # {ctrler_0 : [0, 0, 0, 0], ... }

        # 횟수 카운트
        self.num_of_initializations = {}    # {ctrler_0 : [0, 0, 0, 0], ... }
        self.num_of_movements = {}          # {ctrler_0 : [0, 0, 0, 0], ... }

        # 최신정보 업데이트를 위한 카운트
        self.info_update_cnt = 0            # 0 ~ 10 사이

        # 준비 여부
        self.ready = False

        # 사용 안함
        self.motionchecks = {}      # {ctrler_0 : ['B', 'S', 'S', 'R'], ... }
        self.stopcheckers = {}      # {ctrler_0 : [1, 0, 1, None], ... }
        
        
        # 타이머 초기화
        self.command_operator_timer = QTimer()
        self.infomation_checker_timer = QTimer()
        
        # 타이머 연결
        self.command_operator_timer.timeout.connect(self.command_operator)
        # self.command_operator_timer.timeout.connect(self.command_operator)
        self.infomation_checker_timer.timeout.connect(self.infomation_checker)
        
    
    # ---------------------------------------------------------------------------------------------------------------------------------
    # 타이머 시작 / 중지
    
    def start_command_operator(self,speed = 30):
        '''명령 Queue 수행 시작'''
        # 이벤트 발생 지연시간 = 연산 속도 역수
        delay_ms = int(1000/speed)
        self.command_operator_timer.start(delay_ms)
    
    def stop_command_operator(self):
        '''명령 Queue 수행 중지'''
        # 수행 = False
        self.command_operator_timer.stop()
        
    def start_infomation_checker(self, speed=30):
        '''컨트롤러 및 스테이지 정보 수집 시작'''
        # 이벤트 발생 지연시간 = 연산 속도 역수
        delay_ms = int(1000/speed)
        self.infomation_checker_timer.start(delay_ms)
    
    def stop_infomation_checker(self):
        '''컨트롤러 및 스테이지 정보 수집 중지'''
        # 명령 체크 = False
        self.infomation_checker_timer.stop()
        
    # ---------------------------------------------------------------------------------------------------------------------------------    
    def command_operator(self):
        '''명령을 직접 수행'''
        # 움직이는 명령어 다 걸어놓고 (컨트롤러 넘버랑 포트 다 기록해야함)
        # start 명령하면 구동하면서 실시간 위치 및 상태 업데이트 하여 emit
        with QMutexLocker(self.mutex):
            # 큐가 비어있으면 그냥 보고만 있고
            if self.command_queue.empty():
                return
            # 명령어 수행 중이거나 업데이트 하는 중이라면 넘어가고
            
            if self.excuting_command or any(self.ctrler_updating.values()):
                return
            
            # 여기서 명령 꺼내 두고, excuting_command True 후 락 해제
            order:StageCommand = self.command_queue.get()
            order:dict = order.__dict__
            
            self.excuting_command = True
            
        
        try:
            # 명령어가 없으면 패스
            if not order:
                return
            # 명령어 수행단

            if order['command'] == 'connect':
                # 컨트롤러를 PC에 연결
                if (str(order['executor']).split('_')[0].lower() == 'controller') & (str(order['target']).lower() == 'pc'):
                    
                    # 오더 딕셔너리로부터 controller_'번호' 얻어오기
                    executor_controller_num = str(order['executor']).rsplit('_', 1)[-1]
                    
                    executor_controller_options : dict
                    executor_controller_options = order['connection']
                    executor_controller_old = executor_controller_options['old']
                    executor_controller_sim = executor_controller_options['simulated']
                    
                    # PC와 연결
                    self.connect_to_PC(num= executor_controller_num,
                                       old=executor_controller_old,
                                       simulated= executor_controller_sim)
                    
                # 스테이지를 컨트롤러_번호_포트에 연결
                elif (type(order['executor']).__name__.lower() == 'stage') & (str(order['target']).split('_')[0].lower() == 'controller'):
                    executor_stage : Stage
                    executor_stage = order['executor']
                    # executor_stage.stage_simulation  = order['simulated']
                    
                    _, target_controller_num, target_controller_port = str(order['target']).split('_')
                    
                    self.connect_with_stage(num= int(target_controller_num),
                                            port= int(target_controller_port),
                                            stage= executor_stage)
                    
                    print(f'{type(executor_stage).__name__}를 {target_controller_num}번 컨트롤러의 {target_controller_port}번 포트에 연결')
                    
                    self.signal_data.emit(self.stages,
                                          self.statuses,
                                          self.steps,
                                          self.positions,
                                          self.motions)
                else:
                    # print('안돼 돌아가!')
                    pass
                    
            elif order['command'] == 'disconnect':
                # 컨트롤러를 PC에서 연결 해제
                if (str(order['executor']).split('_')[0].lower() == 'controller') & (str(order['target']).lower() == 'pc'):
                    executor_controller_num = str(order['executor']).rsplit('_', 1)[-1]
                    
                    self.disconnect_to_PC(num=executor_controller_num)
                    
                # 스테이지를 컨트롤러_번호_포트에서 연결 해제
                elif (type(order['executor']).__name__.lower() == 'stage') & (str(order['target']).split('_')[0].lower() == 'controller'):
                    executor_stage : Stage
                    executor_stage = order['executor']
                    
                    self.disconnect_with_stage(stage= executor_stage)
            
            # 연결경로 변경 명령
            elif order['command'] == 'change':
                # 컨트롤러의 네트워크 설정값 변경
                if (str(order['executor']).split('_')[0].lower() == 'controller')&(str(order['target'])!=None):
                    
                    # 오더 딕셔너리로부터 controller_'번호' 얻어오기
                    executor_controller_num = str(order['executor']).rsplit('_', 1)[-1]
                    
                    executor_controller_old = self.ctrler_old_ver[f'ctrler_{executor_controller_num}']
                    executor_controller_sim = self.ctrler_simulated[f'ctrler_{executor_controller_num}']
                    
                    executor_controller_options : dict
                    executor_controller_options = order['connection']
                    
                    
                    target_change_setting = str(order['target']).lower()
                    # 구형 컨트롤러가 아닐 때
                    if not executor_controller_old:
                        if target_change_setting == 'ip':
                            target_cmd = 'IP' # 컨트롤러한테 보낼 명령어
                            # 변경할 IP주소 값
                            executor_controller_changed_value = executor_controller_options['changed_ip']
                            
                        elif target_change_setting == 'subnetmask':
                            target_cmd = 'SM' # 컨트롤러한테 보낼 명령어
                            # 변경할 서브넷마스크 값
                            executor_controller_changed_value = executor_controller_options['changed_subnetmask']
                            
                        elif target_change_setting == 'gateway':
                            target_cmd = 'GW' # 컨트롤러한테 보낼 명령어
                            # 변경할 게이트웨이 값
                            executor_controller_changed_value = executor_controller_options['changed_gateway']
                            
                        elif target_change_setting == 'port':
                            target_cmd = 'PORT' # 컨트롤러한테 보낼 명령어
                            # 변경할 네트워크 포트 값 - 5001 고정(기능 구현만 해둠)
                            executor_controller_changed_value = executor_controller_options['changed_port']
                        
                        # 네트워크 설정값 변경 실행
                        self.change_network_setting(num=executor_controller_num,
                                                    target= target_cmd,
                                                    changed_value=executor_controller_changed_value)
                        
                    else:
                        pass
                
                # 연결된 스테이지를 컨트롤러_번호_포트로 변경
                elif (type(order['executor']).__name__.lower() == 'stage') & (str(order['target']).split('_')[0].lower() == 'controller'):
                    executor_stage : Stage
                    executor_stage = order['executor']
                    # executor_stage.stage_simulation  = order['simulated']
                    
                    _, target_controller_num, target_controller_port = str(order['target']).split('_')
                    
                    # 연결 해제(연결 값: 컨트롤러 번호, 포트 초기화)
                    self.disconnect_with_stage(stage= executor_stage)
                    
                    # 다시 연결
                    self.connect_with_stage(num= int(target_controller_num),
                                            port= int(target_controller_port),
                                            stage= executor_stage)
                    
                    print(f'{type(executor_stage).__name__}를 {target_controller_num}번 컨트롤러의 {target_controller_port}번 포트로 연결 변경')

            
            # 동작 명령어면 그 메세지 주고 오퍼레이터 끝
            elif order['command'] == 'move':
                executor_stage : Stage
                executor_stage = order['executor']
                executor_stage_options = order['movement']
                
                executor_stage_delayms = executor_stage_options['operation_delay_ms']
                
                # QTest.qWait(executor_stage_delayms)
                
                self.move_stage(stage = executor_stage,
                                mode = executor_stage_options['mode'],
                                speedLv = executor_stage_options['speed'],
                                coordinate = executor_stage_options['value'],
                                distance = executor_stage_options['value'],
                                direction = executor_stage_options['direction'])
                
                self.operation_queue.put((executor_stage.ctrler_num,
                                          executor_stage.ctrler_port,
                                          executor_stage_delayms))
            
            # 동작시작 명령어면 그 메세지 주고 -> 인포메이션 체커 활성화, 다음 명령 큐 실행 정지
            elif order['command'] == 'start':
                # 동작 명령 받은 컨트롤러 세트 - 중복없는 정렬된 리스트
                
                for i in range(self.operation_queue.qsize()):
                    
                    operating_target = self.operation_queue.get()
                    
                    # 명령받은 컨트롤러 번호를 추가(set형이므로 중복 제거됨)
                    self.updating_controllers.add(operating_target[0]) 
                    
                    # 딜레이 있으면 기다렸다가
                    QTest.qWait(operating_target[2])
                    # 해당 포트 전원 켜고
                    # self.send(num = operating_target[0], msg = f'{operating_target[1]}ST0')
                    # 동작명령 수행하라고 보내기
                    self.send_recv(num = operating_target[0], msg = f'{operating_target[1]}G')

                with QMutexLocker(self.mutex):
                    # 동작 명령을 받은 컨트롤러 내림차순 리스트
                    # 해당 컨트롤러 지금 업데이트 중이라고 변경
                    for num in sorted(self.updating_controllers):
                        self.ctrler_updating[f'ctrler_{num}'] = True
                    # 전체 컨트롤러 중 각각의 상태 나타내기 위함
                    # 개별로 True/False 확인해서 변경하진 않음
            
            elif order['command'] == 'initialize':
                # PC-컨트롤러-스테이지 첫 연결 후 위치 초기화
                # 연결 직후에 자기 위치를 모르니까 반드시 필요
                
                # 컨트롤러 전체 또는 특정 스테이지만
                if (str(order['executor']).split('_')[0].lower() == 'controller'):
                    
                    target_controller_num = int(str(order['executor']).split('_')[-1])
                    
                    # 컨트롤러 전체 최초 초기화
                    self.initialize_ctrler(num= target_controller_num)
                    
                    self.operation_queue.put((target_controller_num, 0, 0))
                    
                elif (type(order['executor']).__name__.lower() == 'stage'):
                    executor_stage : Stage
                    executor_stage = order['executor']
                    
                    # 특정 스테이지만 최초 초기화
                    self.initialize_stage(stage= executor_stage)
                    
                    self.operation_queue.put((executor_stage.ctrler_num, executor_stage.ctrler_port, 0))
                
                # fastprint(f'초기화 횟수 : {self.num_of_initializations}\n')
            
            elif order['command'] == 'home':
                # 컨트롤러 전체 또는 특정 스테이지만
                if (str(order['executor']).split('_')[0].lower() == 'controller'):
                    # 컨트롤러 전체에 연결된 스테이지들을 좌표 0으로 보내기
                    pass
                elif (type(order['executor']).__name__.lower() == 'stage'):

                    self.command_queue.put(StageCommand(executor=order['executor'],
                                                   command='move',
                                                   movement={'mode': 'coordinate',
                                                             'speed' : 10,
                                                             'value' : 0},
                                                   message='stage home order'))
                    
                    
                    self.command_queue.put(StageCommand(command='start',
                                                   message='stage home start'))
                    
                    self.command_queue.put(StageCommand(executor=order['executor'],
                                                   command='initialize'))
                    
                    self.command_queue.put(StageCommand(command='refresh'))
                    
                # 위치 0으로 보내고 초기화
                    # 0보다 살짝 앞으로 보내기
                    # 초기화
                    
                    # 업데이팅 활성화
                for num in sorted(self.updating_controllers):
                    self.ctrler_updating[f'ctrler_{num}'] = True
                    
            elif order['command'] == 'refresh':
                # 조그이동 / 초기화 / 원점위치 명령 후 수동 최신화
                for _ in range(self.operation_queue.qsize()):
                    # print(_)
                    operating_target = self.operation_queue.get()
                    
                    # 명령받은 컨트롤러 번호를 추가(set형이므로 중복 제거됨)
                    self.updating_controllers.add(operating_target[0])
                
                with QMutexLocker(self.mutex):
                    # 동작 명령을 받은 컨트롤러 내림차순 리스트
                    # self.updating_controllers = sorted(self.updating_controllers) # list
                    
                    # 해당 컨트롤러 지금 업데이트 중이라고 변경
                    for num in sorted(self.updating_controllers):
                        self.ctrler_updating[f'ctrler_{num}'] = True
                    
                    # 전체 컨트롤러 중 각각의 상태 나타내기 위함
                    # 개별로 True/False 확인해서 변경하진 않음
                    # print(self.ctrler_updating)
        
        finally:
            # 무조건 플래그를 리셋
            with QMutexLocker(self.mutex):
                self.excuting_command = False
    
    def past_command_operator(self):
        '''명령을 직접 수행'''
        # 움직이는 명령어 다 걸어놓고 (컨트롤러 넘버랑 포트 다 기록해야함)
        # start 명령하면 구동하면서 실시간 위치 및 상태 업데이트 하여 emit
        
        with QMutexLocker(self.mutex):
            # 큐가 비어있으면 그냥 보고만 있고
            if self.command_queue.empty():
                order= {}
                
                # 몇초 뒤에 다시 체크해
                # time.sleep(0.01) 
                # QTest.qWait(10) # 10ms
                
            # 큐가 있으면 명령어 가져오기
            # 업데이트로 인한 큐 명령 수행 금지 오더가 없으면
            else:
                # print(self.command_queue.qsize())
                order: StageCommand
                
                # 모든 컨트롤러가 업데이트 중이 아니면
                if (not any(self.ctrler_updating.values())):
                    # 큐에서 명령 딕셔너리 빼옴
                    order= self.command_queue.get()
                
                    # 데이터클래스 변수 딕셔너리화 :
                    # asdict은 깊은 복사 수행 -> 클래스 변수 값을 반영하지 못함
                    # order = asdict(order)
                    
                    # __dict__은 얕은 복사 수행 -> 클래스 변수 값을 반영함
                    order= order.__dict__
                    
                    # 명령 타입에 대한 힌트 : 딕셔너리
                    order: dict
                    # 명령 번호 및 타임스탬프 입력
                    # 명령 딕셔너리에 명령 번호와 타임스탬프 추가
                    
                    # order['number'] = 0
                    # order['timestamp'] = 0
                    
                    # print(order)
                else:
                    order = {}
                    pass
            
        # 중복 QMutex 방지 조건문 분리-----------------

        # 명령 수행단
        
        # 모든 컨트롤러가 업데이트 중이 아니고 & 가져온 명령어가 빈 딕셔너리가 아닐 때
        if (not any(self.ctrler_updating.values())) & (len(order.keys()) != 0):
            # 명령 실행
            # 명령어 순서를 로그로 남겨놓기 -> 이 조건문 제일 마지막에 로그 추가 예정
            
            # 컨트롤러를 PC에 / 자동 스테이지를 컨트롤러에 연결
            if order['command'] == 'connect':
                # 컨트롤러를 PC에 연결
                if (str(order['executor']).split('_')[0].lower() == 'controller') & (str(order['target']).lower() == 'pc'):
                    
                    # 오더 딕셔너리로부터 controller_'번호' 얻어오기
                    executor_controller_num = str(order['executor']).rsplit('_', 1)[-1]
                    
                    executor_controller_options : dict
                    executor_controller_options = order['connection']
                    executor_controller_old = executor_controller_options['old']
                    executor_controller_sim = executor_controller_options['simulated']
                    
                    # PC와 연결
                    self.connect_to_PC(num= executor_controller_num,
                                       old=executor_controller_old,
                                       simulated= executor_controller_sim)
                    
                # 스테이지를 컨트롤러_번호_포트에 연결
                elif (type(order['executor']).__name__.lower() == 'stage') & (str(order['target']).split('_')[0].lower() == 'controller'):
                    executor_stage : Stage
                    executor_stage = order['executor']
                    # executor_stage.stage_simulation  = order['simulated']
                    
                    _, target_controller_num, target_controller_port = str(order['target']).split('_')
                    
                    self.connect_with_stage(num= int(target_controller_num),
                                            port= int(target_controller_port),
                                            stage= executor_stage)
                    
                    print(f'{type(executor_stage).__name__}를 {target_controller_num}번 컨트롤러의 {target_controller_port}번 포트에 연결')
                    
                    self.signal_data.emit(self.stages,
                                          self.statuses,
                                          self.steps,
                                          self.positions,
                                          self.motions)
                else:
                    # print('안돼 돌아가!')
                    pass
                    
            elif order['command'] == 'disconnect':
                # 컨트롤러를 PC에서 연결 해제
                if (str(order['executor']).split('_')[0].lower() == 'controller') & (str(order['target']).lower() == 'pc'):
                    executor_controller_num = str(order['executor']).rsplit('_', 1)[-1]
                    
                    self.disconnect_to_PC(num=executor_controller_num)
                    
                # 스테이지를 컨트롤러_번호_포트에서 연결 해제
                elif (type(order['executor']).__name__.lower() == 'stage') & (str(order['target']).split('_')[0].lower() == 'controller'):
                    executor_stage : Stage
                    executor_stage = order['executor']
                    
                    self.disconnect_with_stage(stage= executor_stage)
            
            # 연결경로 변경 명령
            elif order['command'] == 'change':
                # 컨트롤러의 네트워크 설정값 변경
                if (str(order['executor']).split('_')[0].lower() == 'controller')&(str(order['target'])!=None):
                    
                    # 오더 딕셔너리로부터 controller_'번호' 얻어오기
                    executor_controller_num = str(order['executor']).rsplit('_', 1)[-1]
                    
                    executor_controller_old = self.ctrler_old_ver[f'ctrler_{executor_controller_num}']
                    executor_controller_sim = self.ctrler_simulated[f'ctrler_{executor_controller_num}']
                    
                    executor_controller_options : dict
                    executor_controller_options = order['connection']
                    
                    
                    target_change_setting = str(order['target']).lower()
                    # 구형 컨트롤러가 아닐 때
                    if not executor_controller_old:
                        if target_change_setting == 'ip':
                            target_cmd = 'IP' # 컨트롤러한테 보낼 명령어
                            # 변경할 IP주소 값
                            executor_controller_changed_value = executor_controller_options['changed_ip']
                            
                        elif target_change_setting == 'subnetmask':
                            target_cmd = 'SM' # 컨트롤러한테 보낼 명령어
                            # 변경할 서브넷마스크 값
                            executor_controller_changed_value = executor_controller_options['changed_subnetmask']
                            
                        elif target_change_setting == 'gateway':
                            target_cmd = 'GW' # 컨트롤러한테 보낼 명령어
                            # 변경할 게이트웨이 값
                            executor_controller_changed_value = executor_controller_options['changed_gateway']
                            
                        elif target_change_setting == 'port':
                            target_cmd = 'PORT' # 컨트롤러한테 보낼 명령어
                            # 변경할 네트워크 포트 값 - 5001 고정(기능 구현만 해둠)
                            executor_controller_changed_value = executor_controller_options['changed_port']
                        
                        # 네트워크 설정값 변경 실행
                        self.change_network_setting(num=executor_controller_num,
                                                    target= target_cmd,
                                                    changed_value=executor_controller_changed_value)
                        
                    else:
                        pass
                
                # 연결된 스테이지를 컨트롤러_번호_포트로 변경
                elif (type(order['executor']).__name__.lower() == 'stage') & (str(order['target']).split('_')[0].lower() == 'controller'):
                    executor_stage : Stage
                    executor_stage = order['executor']
                    # executor_stage.stage_simulation  = order['simulated']
                    
                    _, target_controller_num, target_controller_port = str(order['target']).split('_')
                    
                    # 연결 해제(연결 값: 컨트롤러 번호, 포트 초기화)
                    self.disconnect_with_stage(stage= executor_stage)
                    
                    # 다시 연결
                    self.connect_with_stage(num= int(target_controller_num),
                                            port= int(target_controller_port),
                                            stage= executor_stage)
                    
                    print(f'{type(executor_stage).__name__}를 {target_controller_num}번 컨트롤러의 {target_controller_port}번 포트로 연결 변경')

            
            # 동작 명령어면 그 메세지 주고 오퍼레이터 끝
            elif order['command'] == 'move':
                executor_stage : Stage
                executor_stage = order['executor']
                executor_stage_options = order['movement']
                
                executor_stage_delayms = executor_stage_options['operation_delay_ms']
                
                # QTest.qWait(executor_stage_delayms)
                
                self.move_stage(stage = executor_stage,
                                mode = executor_stage_options['mode'],
                                speedLv = executor_stage_options['speed'],
                                coordinate= executor_stage_options['value'],
                                distance = executor_stage_options['value'],
                                direction = executor_stage_options['direction'])
                
                self.operation_queue.put((executor_stage.ctrler_num,
                                          executor_stage.ctrler_port,
                                          executor_stage_delayms))
            
            # 동작시작 명령어면 그 메세지 주고 -> 인포메이션 체커 활성화, 다음 명령 큐 실행 정지
            elif order['command'] == 'start':
                # 동작 명령 받은 컨트롤러 세트 - 중복없는 정렬된 리스트
                
                for i in range(self.operation_queue.qsize()):
                    
                    operating_target = self.operation_queue.get()
                    
                    # 명령받은 컨트롤러 번호를 추가(set형이므로 중복 제거됨)
                    self.updating_controllers.add(operating_target[0]) 
                    
                    # 딜레이 있으면 기다렸다가
                    QTest.qWait(operating_target[2])
                    # 해당 포트 전원 켜고
                    # self.send(num = operating_target[0], msg = f'{operating_target[1]}ST0')
                    # 동작명령 수행하라고 보내기
                    self.send_recv(num = operating_target[0], msg = f'{operating_target[1]}G')

                with QMutexLocker(self.mutex):
                    # 동작 명령을 받은 컨트롤러 내림차순 리스트
                    # 해당 컨트롤러 지금 업데이트 중이라고 변경
                    for num in sorted(self.updating_controllers):
                        self.ctrler_updating[f'ctrler_{num}'] = True
                    # 전체 컨트롤러 중 각각의 상태 나타내기 위함
                    # 개별로 True/False 확인해서 변경하진 않음
            
            elif order['command'] == 'initialize':
                # PC-컨트롤러-스테이지 첫 연결 후 위치 초기화
                # 연결 직후에 자기 위치를 모르니까 반드시 필요
                
                # 컨트롤러 전체 또는 특정 스테이지만
                if (str(order['executor']).split('_')[0].lower() == 'controller'):
                    
                    target_controller_num = int(str(order['executor']).split('_')[-1])
                    
                    # 컨트롤러 전체 최초 초기화
                    self.initialize_ctrler(num= target_controller_num)
                    
                    self.operation_queue.put((target_controller_num, 0, 0))
                    
                elif (type(order['executor']).__name__.lower() == 'stage'):
                    executor_stage : Stage
                    executor_stage = order['executor']
                    
                    # 특정 스테이지만 최초 초기화
                    self.initialize_stage(stage= executor_stage)
                    
                    self.operation_queue.put((executor_stage.ctrler_num, executor_stage.ctrler_port, 0))
                
                # fastprint(f'초기화 횟수 : {self.num_of_initializations}\n')
            
            elif order['command'] == 'home':
                # 컨트롤러 전체 또는 특정 스테이지만
                if (str(order['executor']).split('_')[0].lower() == 'controller'):
                    # 컨트롤러 전체에 연결된 스테이지들을 좌표 0으로 보내기
                    pass
                elif (type(order['executor']).__name__.lower() == 'stage'):

                    self.command_queue.put(StageCommand(executor=order['executor'],
                                                   command='move',
                                                   movement={'mode': 'coordinate',
                                                             'speed' : 10,
                                                             'value' : 0},
                                                   message='stage home order'))
                    
                    
                    self.command_queue.put(StageCommand(command='start',
                                                   message='stage home start'))
                    
                    self.command_queue.put(StageCommand(executor=order['executor'],
                                                   command='initialize'))
                    
                    self.command_queue.put(StageCommand(command='refresh'))
                    
                # 위치 0으로 보내고 초기화
                    # 0보다 살짝 앞으로 보내기
                    # 초기화
                    
                    # 업데이팅 활성화
                for num in sorted(self.updating_controllers):
                    self.ctrler_updating[f'ctrler_{num}'] = True
                    
            elif order['command'] == 'refresh':
                # 조그이동 / 초기화 / 원점위치 명령 후 수동 최신화
                for _ in range(self.operation_queue.qsize()):
                    # print(_)
                    operating_target = self.operation_queue.get()
                    
                    # 명령받은 컨트롤러 번호를 추가(set형이므로 중복 제거됨)
                    self.updating_controllers.add(operating_target[0])
                
                with QMutexLocker(self.mutex):
                    # 동작 명령을 받은 컨트롤러 내림차순 리스트
                    # self.updating_controllers = sorted(self.updating_controllers) # list
                    
                    # 해당 컨트롤러 지금 업데이트 중이라고 변경
                    for num in sorted(self.updating_controllers):
                        self.ctrler_updating[f'ctrler_{num}'] = True
                    
                    # 전체 컨트롤러 중 각각의 상태 나타내기 위함
                    # 개별로 True/False 확인해서 변경하진 않음
                    # print(self.ctrler_updating)
                
        # 그 외 유효하지 않은 명령 or 상태
        else:
            # print('컷!')
            pass

        # 시스템 로그 추가
        
        
    def infomation_checker(self):
        '''스테이지 / 컨트롤러 정보 수집 여부 체크'''
        with QMutexLocker(self.mutex):
            # 중복 참조 막기 위한 뮤텍스
            # 여기서 값을 가져와서 일단 복사

            # 구동 명령을 받아 업데이팅 해야하는 컨트롤러 리스트
            # 명령 단계에서 중복 제거 및 정렬 완료된 리스트형
            updating_controller_list = self.updating_controllers
            
            # 기존 값 가져오기
            # entire_statuses = self.statuses
            # entire_steps = self.steps
            entire_positions = self.positions
            entire_motions = self.motions
            
        # 한개의 컨트롤러라도 업데이트 중이라면    
        if any(self.ctrler_updating.values()):
            # 동작 명령 받은 컨트롤러
            for updating_controller_num in updating_controller_list:
                self.load_statuses_of_stages(num= updating_controller_num)
                self.load_motions_of_stages(num= updating_controller_num)
                self.load_steps_positions_of_stages(num= updating_controller_num)

            # 그 중에서
            # 모든 컨트롤러의 모션 딕셔너리가 비어있지 않고 (키-값 쌍이 있으면 True)
            # 모든 컨트롤러의 모션 딕셔너리 벨류 리스트에서 'B'가 단 한개도 없을 때
            if entire_motions and (not any('B' in stage_motions for stage_motions in entire_motions.values())):
                # 몇번 더 업데이트 필요한데
                self.info_update_cnt += 1
                # fastprint(f'동작 종료 후 업뎃 {self.info_update_cnt}번째\n')

                if self.info_update_cnt > 10:
                    # 업데이트 중지 - 모든 컨트롤러의 업데이트 상태 False
                    for updating_controller_num in updating_controller_list:
                        self.ctrler_updating[f'ctrler_{updating_controller_num}'] = False
                    
                        # 동작이 끝난 모든 컨트롤러의 모든 포트 전원 끄기
                        self.send_recv(num= updating_controller_num, msg='@ST1')

                    # 업데이트 카운터 초기화
                    self.info_update_cnt = 0

                    # 동작 명령 받은 컨트롤러 번호 세트 다시 초기화
                    self.updating_controllers = set()
            
            
            # fastprint(f'동작 : {entire_motions}\n')
            # fastprint(f'위치 : {entire_positions}\n')
            # 외부로 보내기
            self.signal_status.emit(self.ctrler_orderable,
                                    self.ctrler_updating,
                                    self.num_of_initializations,
                                    self.num_of_movements,
                                    self.ready)
            
            self.signal_data.emit(self.stages,
                                self.statuses,
                                self.steps,
                                self.positions,
                                self.motions)


        # 모든 컨트롤러가 명령 가능하고
        order_ready = all(self.ctrler_orderable.values())
        # 모든 업데이트가 끝나고        
        update_ready = all(not v for v in self.ctrler_updating.values())
        # 모든 축이 한번 이상 초기화를 했을 때
        init_ready = all( v >= 1
                        for values in self.num_of_initializations.values()
                        for v in values
                        if v is not None)
        
        # 스테이지 컨트롤러 준비완료
        self.ready = order_ready and update_ready and init_ready

        # # 외부로 보내기
        # self.signal_status.emit(self.ctrler_orderable,
        #                         self.ctrler_updating,
        #                         self.num_of_initializations,
        #                         self.num_of_movements,
        #                         self.ready)
        
        # self.signal_data.emit(self.stages,
        #                       self.statuses,
        #                       self.steps,
        #                       self.positions,
        #                       self.motions)
    
    # ---------------------------------------------------------------------------------------------------------------------------------
    def switch_to_simulated(self, num: int, simulated: bool):
        '''지정된 컨트롤러의 실제/더미 상태 확인 및 변경'''
        with QMutexLocker(self.mutex):
        # 딕셔너리 안에 시뮬레이션 키 값이 없으면 추가
            if not f'ctrler_{num}' in self.ctrler_simulated:
                self.ctrler_simulated[f'ctrler_{num}'] = simulated
            
            # 딕셔너리 안에 시뮬레이션 키 값이 있으면 변경
                # 이미 있는 값과 다를 경우 변경
                # 이미 있는 값과 같을 경우 패스
            else:
                if self.ctrler_simulated[f'ctrler_{num}'] != simulated:
                    self.ctrler_simulated[f'ctrler_{num}'] = simulated
                else:
                    pass
    
    def switch_to_simulating(self, num: int, simulating: bool):
        '''지정된 컨트롤러의 값 반환 시뮬레이팅 확인 및 변경'''
        # 딕셔너리 안에 시뮬레이팅 키 값이 없으면 추가
        if not f'ctrler_{num}' in self.ctrler_simulating:
            self.ctrler_simulating[f'ctrler_{num}'] = simulating
        
        # 딕셔너리 안에 시뮬레이팅 키 값이 있으면 변경
            # 이미 있는 값과 다를 경우 변경
            # 이미 있는 값과 같을 경우 패스
        else:
            if self.ctrler_simulating[f'ctrler_{num}'] != simulating:
                self.ctrler_simulating[f'ctrler_{num}'] = simulating
            else:
                pass
            
    def connect_to_PC(self, num: int, old: bool = False, simulated = False, simulating= False):
        '''지정된 컨트롤러와 PC를 연결'''
        with QMutexLocker(self.mutex):
            # 컨트롤러 버전 정보
            self.ctrler_old_ver[f'ctrler_{num}'] = old
            
            # 시뮬레이션 딕셔너리 값 입력
            self.ctrler_simulated[f'ctrler_{num}'] = simulated
            self.ctrler_simulating[f'ctrler_{num}'] = simulating
            
            # 스테이지 초기값 list 생성
            # 딕셔너리 값 크기 부여
            self.stages[f'ctrler_{num}']   = [None for i in range(self.ctrler_ports)]
            self.statuses[f'ctrler_{num}'] = [None for i in range(self.ctrler_ports)]
            self.motions[f'ctrler_{num}']  = [None for i in range(self.ctrler_ports)]
            self.steps[f'ctrler_{num}']    = [None for i in range(self.ctrler_ports)]
            
            self.motionchecks[f'ctrler_{num}'] = [None for i in range(self.ctrler_ports)]
            self.stopcheckers[f'ctrler_{num}'] = [None for i in range(self.ctrler_ports)]
            self.positions[f'ctrler_{num}']    = [None for i in range(self.ctrler_ports)]
            
            self.num_of_initializations[f'ctrler_{num}'] = [None for i in range(self.ctrler_ports)]
            self.num_of_movements[f'ctrler_{num}'] = [None for i in range(self.ctrler_ports)]

            try:
                if not self.ctrler_simulated[f'ctrler_{num}']:
                    skt = sk.socket(sk.AF_INET, sk.SOCK_STREAM)
                    skt.settimeout(0.5)
                    # 현재 임시 조치 중(네트워크 스위치 192.168.100.X 대역 사용 중)
                    ip = f'192.168.100.{122+int(num)}'
                    port = 5001
                    
                    skt.connect((ip, port))
                    
                    QTest.qWait(20) # 20ms
                    # QThread().msleep(20)
                    # time.sleep(0.02)
                    
                    # 실제 연결 로그 발송
                    fastprint(f'{num}번 제어기와 연결\n')
                    
                else:
                    skt = 'simulation'
                    ip  = 'simulation'
                    port= 'simulation'
                    
                    QTest.qWait(20) # 20ms
                    # QThread().msleep(20)
                    # time.sleep(0.02)
                    
                    fastprint(f'{num}번 더미 제어기와 가상연결\n')    
                    # 가상 연결 로그 발송
                
                # 컨트롤러 정보 딕셔너리 값 크기 부여
                self.ctrler_sockets[f'ctrler_{num}'] = skt
                self.ctrler_ipadress[f'ctrler_{num}'] = ip
                
                # 구동 명령 가능
                # self.ctrler_orderable[f'ctrler_{num}'] = True
                
            except:
                # 연결된 컨트롤러만 딕셔너리 키 생성됨
                self.ctrler_sockets[f'ctrler_{num}'] = None
                self.ctrler_ipadress[f'ctrler_{num}'] = None
                
                # 구동명령 불가
                # self.ctrler_orderable[f'ctrler_{num}'] = False
                
                fastprint(f'실제 연결 실패로 {num}번 더미 제어기와 가상연결\n')
                pass

            # 연결한 컨트롤러 수 만큼 최초값 부여
            self.ctrler_orderable[f'ctrler_{num}'] = True   # 구동명령 가능
            self.ctrler_updating[f'ctrler_{num}'] = False   # 업데이트 중 아님
            # self.ctrler_ready[f'ctrler_{num}'] = False      # 준비되지 않음

    def disconnect_to_PC(self, num: int):
        '''지정 컨트롤러의 소켓통신 연결종료'''
        
        # 업데이트 중 아님
        self.ctrler_updating[f'ctrler_{num}'] = False
        
        # 동작 명령 불가
        self.ctrler_orderable[f'ctrler_{num}'] = False
        
        if not self.ctrler_simulated[f'ctrler_{num}']:
            try:
                self.ctrler_sockets[f'ctrler_{num}'].close()
                fastprint(f'{num}번 제어기와 연결종료\n')
            except:
                fastprint(f'disconnect error: Controller {num}\n')
                pass
        else:
            self.ctrler_sockets[f'ctrler_{num}'] = None
            
            fastprint(f'{num}번 더미 제어기와 가상 연결해제\n')
    
    def change_network_setting(self, num:int, target:str= None, changed_value:str= None):
        '''
        *** 구형 컨트롤러 사용 불가 ***
        컨트롤러 네트워크 설정값 변경
        '''
        # 구형 컨트롤러가 아니고 변경 값이 None이 아닐 때
        if (not self.ctrler_old_ver[f'ctrler_{num}']) & (changed_value != None):
            
            msg_cmd = target # IP / SM / GW / PORT
            msg_value = changed_value.replace('.',',')  # 입력받은 값의 온점을 쉼표로 대체
                                                        # 온점 없으면(PORT) 원래 값 반환
            
            print(f'{msg_cmd}{msg_value}')
            
            data = self.send_recv(num, msg=f'{msg_cmd}{msg_value}')
            
            print(data)
    
    def connect_with_stage(self, num: int, port: int, stage: Stage= None):
        '''지정 스테이지를 지정 컨트롤러에 연결'''
        with QMutexLocker(self.mutex):
            if stage != None:
                stage.ctrler_num = num
                stage.ctrler_port = port
                # stage.ctrler_socket_id = self.ctrler_sockets[f'ctrler_{num}']
                
                self.stages[f'ctrler_{num}'][port-1] = stage
                if stage.stage_type == 'R':
                    self.send_recv(num=stage.ctrler_num,msg=f'{stage.ctrler_port}HLD3')
                    # 회전 스테이지는 리미트가 하나라서 전체 미사용으로 해야 구동함

                # 카운팅 생성
                self.num_of_initializations[f'ctrler_{num}'][port-1] = 0
                self.num_of_movements[f'ctrler_{num}'][port-1] = 0
                
    def disconnect_with_stage(self, stage: Stage= None):
        '''지정 스테이지를 지정 컨트롤러와 연결 해제'''
        with QMutexLocker(self.mutex):
            if stage != None:  
                self.stages[f'ctrler_{stage.ctrler_num}'][stage.ctrler_port-1] = None
                
                # 기존 컨트롤러 번호, 포트 불러오기
                num = stage.ctrler_num
                port = stage.ctrler_port
                # stage.ctrler_socket_id = None
                print(f'{num}번 컨트롤러로부터 {port}번 스테이지 연결 해제')
                # 카운팅 말소
                self.num_of_initializations[f'ctrler_{num}'][port-1] = None
                self.num_of_movements[f'ctrler_{num}'][port-1] = None
                
                # 번호, 포트 말소
                stage.ctrler_num = None
                stage.ctrler_port = None

                
    # 함수 내 try 구문이 있어야 구/신형 컨트롤러 전부 작동
    # 오류 발생시 except 구문에서 어떤 메세지 내보낼지는 아직 미정
    # 오류 발생시 pass 넣어야 스레드 재실행 가능
    
    # 명령 큐 수행 단에서 시그널 emit 예정
    # 한 컨트롤러에 연결된 모든 스테이지의 데이터
    def load_statuses_of_stages(self, num: int)-> list:
        '''지정 컨트롤러에 연결된 모든 스테이지 상태 불러오기'''
        
        with QMutexLocker(self.mutex):
            if self.ctrler_orderable[f'ctrler_{num}']: # 해당 컨트롤러가 명령 가능할 때
                if not self.ctrler_simulating[f'ctrler_{num}']: # 해당 컨트롤러가 시뮬레이팅 중이 아닐 때
                    try:
                        statmsg = str(self.send_recv(num, msg='@STATUS'))         # 반환 데이터 원문
                        statidx = statmsg.find('STATUS')
                        statlist = list(map(str,statmsg[statidx+7:].split(',')))    # 포트 별로 나누기
                        
                        for _ in range(self.ctrler_ports): # 고정값 4
                            stat = statlist[_].replace('\r','').replace('_','')     # 개행문자, 언더바 제거
                            stat = ''.join(reversed(stat))
                            # 코드길이가 20개 맞는지 확인 - 첫번째 메세지는 21로 나오더라
                            if len(stat) != 20:
                                stat = stat[:-1]
                                
                            # 스테이지 클래스 함수로 상태 정보 변환
                            status_updated_stage : Stage
                            status_updated_stage = self.stages[f'ctrler_{num}'][_]
                            if status_updated_stage != None:
                                self.statuses[f'ctrler_{num}'][_] = stat
                                status_updated_stage.statuscode_converter(statuscode = stat)
                            else:
                                self.statuses[f'ctrler_{num}'][_] = None
                        
                        return self.statuses[f'ctrler_{num}']
                
                    except:
                        # 컨트롤러와 연결된 스테이지가 없을 때
                        # self.stages[ctrler_0][0] = None 일 때
                        print(f'엄')
                        pass
                    
                else: # 해당 컨트롤러가 시뮬레이팅 중일 때
                    # 시뮬레이팅 값 반환
                    # print('뭐야')
                    pass
                
            else: # 해당 컨트롤러가 명령 불가능할 때
                print('명령 수행 불가_1')
                
                pass
    
    def load_steps_positions_of_stages(self, num: int)-> tuple[list, list]:
        '''지정 컨트롤러에 연결된 모든 스테이지 스탭과 위치 불러오기'''
        with QMutexLocker(self.mutex):
            if self.ctrler_orderable[f'ctrler_{num}']: # 해당 컨트롤러가 명령 가능할 때
                if not self.ctrler_simulating[f'ctrler_{num}']: # 해당 컨트롤러가 시뮬레이팅 중이 아닐 때
                    try:
                        stepsmsg = str(self.send_recv(num,msg='@P'))[:-1]        # 반환 데이터 원문
                        stepsidx = stepsmsg.find('POS')
                        steplist = list(map(int,stepsmsg[stepsidx+3:].split(',')))  # 포트 별로 나누기
                        
                        self.steps[f'ctrler_{num}'] = steplist
                        
                        for _ in range(self.ctrler_ports): # 고정값 4
                            # 컨트롤러에 스탭 리스트 추가
                            self.steps[f'ctrler_{num}'][_] = steplist[_]
                            
                            step_updated_stage : Stage
                            step_updated_stage = self.stages[f'ctrler_{num}'][_]
                            
                            # 스테이지 클래스 함수로 스탭 -> 위치 변환
                            if step_updated_stage != None:
                                step_updated_stage.step = steplist[_]
                                
                                pos = step_updated_stage.convert_step_to_position(steplist[_])
                                step_updated_stage.position = pos
                                self.positions[f'ctrler_{num}'][_] = pos

                            else:
                                self.positions[f'ctrler_{num}'][_] = None

                        return self.steps[f'ctrler_{num}'], self.positions[f'ctrler_{num}']
                
                    except:
                        # 컨트롤러와 연결된 스테이지가 없을 때
                        # self.stages[ctrler_0][0] = None 일 때
                        print('준')
                        pass
                    
                else: # 해당 컨트롤러가 시뮬레이팅 중일 때
                    # 시뮬레이팅 값 반환
                    # print('뭐야')
                    pass
                
            else: # 해당 컨트롤러가 명령 불가능할 때
                print('명령 수행 불가_2')
                
                pass
        
    def load_motions_of_stages(self, num: int)-> list:
        '''지정 컨트롤러에 연결된 모든 스테이지 모션 불러오기'''
        with QMutexLocker(self.mutex):
            if self.ctrler_orderable[f'ctrler_{num}']: # 해당 컨트롤러가 명령 가능할 때
                if not self.ctrler_simulating[f'ctrler_{num}']: # 해당 컨트롤러가 시뮬레이팅 중이 아닐 때
                    try:
                        motionmsg = str(self.send_recv(num, msg='@R'))[:-1]           # 반환 데이터 원문
                        motionidx = motionmsg.find('R:')
                        motionlist = list(map(str,motionmsg[motionidx+2:].split(',')))  # 포트 별로 나누기
                        
                        self.motions[f'ctrler_{num}'] = motionlist
                        
                        for _ in range(self.ctrler_ports): # 고정값 4
                            self.motions[f'ctrler_{num}'][_] = motionlist[_]
                            
                            motion_updated_stage: Stage
                            motion_updated_stage = self.stages[f'ctrler_{num}'][_]
                            
                            # 스테이지 클래스 함수내에 모션 저장
                            if motion_updated_stage != None:
                                motion_updated_stage.motion = motionlist[_]

                        return self.motions[f'ctrler_{num}']
                
                    except:
                        # 컨트롤러와 연결된 스테이지가 없을 때
                        # self.stages[ctrler_0][0] = None 일 때
                        pass
                    
                else: # 해당 컨트롤러가 시뮬레이팅 중일 때
                    # 시뮬레이팅 값 반환
                    # print('뭐야')
                    print('식')
                    pass
                
            else: # 해당 컨트롤러가 명령 불가능할 때
                print('명령 수행 불가_3')
                pass
    
    # 거의 쓸 일 없을텐데 기능적으로는 구현
    def load_status_of_stage(self, stage: Stage= None)-> str:
        '''지정 스테이지 상태 불러오기'''
        with QMutexLocker(self.mutex):
            if stage != None:
                num = stage.ctrler_num
                port = stage.ctrler_port
                
                if self.ctrler_orderable[f'ctrler_{num}']: # 해당 컨트롤러가 명령 가능할 때
                    if not self.ctrler_simulating[f'ctrler_{num}']: # 해당 컨트롤러가 시뮬레이팅 중이 아닐 때
                        try:
                            statmsg = str(self.send_recv(num, msg=f'{port}STATUS'))         # 반환 데이터 원문
                            # print(statmsg)
                            statidx = statmsg.find('STATUS')
                            statlist = list(map(str,statmsg[statidx+7:].split(',')))    # 포트 별로 나누기
                            
                            stat = statmsg[statidx+7:].replace('\r','').replace('_','')     # 개행문자, 언더바 제거
                            stat = ''.join(reversed(stat))
                            
                            if len(stat) != 20:
                                stat = stat[:-1]
                                
                            self.statuses[f'ctrler_{num}'][port-1] = stat
                            
                            stage.status = stat
                            
                            stage.statuscode_converter(statuscode = stat)
                            
                            self.statuses[f'ctrler_{num}'][port-1] = stat
                            
                            return stage.status
                    
                        except:
                            # 최신화 이전 상태 정보 값 반환
                            return self.statuses[f'ctrler_{num}'][port-1]
                        
                    else: # 해당 컨트롤러가 시뮬레이팅 중일 때
                        # 시뮬레이팅 값 반환
                        # print('뭐야')
                        pass
                    
                else: # 해당 컨트롤러가 명령 불가능할 때
                    print('명령 수행 불가_4')
                    
                    pass
    
    def load_step_position_of_stage(self, stage: Stage= None)-> tuple[int, float]:
        '''지정 스테이지 스탭과 위치 불러오기'''
        with QMutexLocker(self.mutex):
            if stage != None:
                num = stage.ctrler_num
                port = stage.ctrler_port
                
                if self.ctrler_orderable[f'ctrler_{num}']: # 해당 컨트롤러가 명령 가능할 때
                    if not self.ctrler_simulating[f'ctrler_{num}']: # 해당 컨트롤러가 시뮬레이팅 중이 아닐 때
                        try:
                            # stepsmsg = str(self.send_recv(num,msg=f'{port}P'))[:-1]        # 반환 데이터 원문
                            stepmsg = str(self.send_recv(num,msg=f'{port}P'))        # 반환 데이터 원문
                            stepsidx = stepmsg.find('POS')
                            
                            step = int(stepmsg[stepsidx+3:])
                            
                            stage.step = int(stepmsg[stepsidx+3:])
                            stage.position = stage.convert_step_to_position(step)
                            
                            self.steps[f'ctrler_{num}'][port-1] = step
                            self.positions[f'ctrler_{num}'][port-1] = stage.position

                            # print(self.positions)
                            
                            return stage.step, stage.position
                    
                        except:
                            # 컨트롤러와 연결된 스테이지가 없을 때
                            # self.stages[ctrler_0][0] = None 일 때
                            print('뭐여')
                            pass
                        
                    else: # 해당 컨트롤러가 시뮬레이팅 중일 때
                        # 시뮬레이팅 값 반환
                        print('뭐야')
                        pass
                    
                else: # 해당 컨트롤러가 명령 불가능할 때
                    print('명령 수행 불가_5')
                    
                    pass
        
    def load_motion_of_stage(self,stage: Stage= None)-> str:
        '''지정 스테이지 모션 불러오기'''
        with QMutexLocker(self.mutex):
            if stage != None:
                num = stage.ctrler_num
                port = stage.ctrler_port
                
                if self.ctrler_orderable[f'ctrler_{num}']: # 해당 컨트롤러가 명령 가능할 때
                    if not self.ctrler_simulating[f'ctrler_{num}']: # 해당 컨트롤러가 시뮬레이팅 중이 아닐 때
                        try:
                            motionmsg = str(self.send_recv(num, msg=f'{port}R'))[:-1]           # 반환 데이터 원문
                            motionidx = motionmsg.find('R:')
                            # motionlist = list(map(str,motionmsg[motionidx+2:].split(',')))  # 포트 별로 나누기
                            motion = motionmsg[motionidx+2:]

                            self.motions[f'ctrler_{num}'][port-1] = motion
                            stage.motion = motion
                            
                            # self.stages[f'ctrler_{num}'][port-1].motion = motion
                            
                            return stage.motion
                    
                        except:
                            # 최신화 이전 상태 정보 값 반환
                            return stage.motion
                        
                    else: # 해당 컨트롤러가 시뮬레이팅 중일 때
                        # 시뮬레이팅 값 반환
                        # print('뭐야')
                        pass
                    
                else: # 해당 컨트롤러가 명령 불가능할 때
                    print('명령 수행 불가_6')
                    
                    pass
        
    # -------------------------------------------------------------------------------------
    def initialize_ctrler(self, num: int):
        '''특정 컨트롤러에 연결된 모든 스테이지 위치 초기화'''
        with QMutexLocker(self.mutex):
            if self.ctrler_orderable[f'ctrler_{num}']: # 해당 컨트롤러가 명령 가능할 때
                if not self.ctrler_simulating[f'ctrler_{num}']: # 해당 컨트롤러가 시뮬레이팅 중이 아닐 때
                    try:
                        # 추가 구동명령 불가
                        self.ctrler_orderable[f'ctrler_{num}'] = False
                        
                        # 구형 스테이지 HM1 - 모터 쪽으로 끝까지 갔다가 홈으로
                        # 구형 스테이지 HM0 - 아주 살짝 움직였다가 홈으로
                        
                        # 신형 스테이지 HM1 - 아주 살짝 움직였다가 홈으로
                        # 신형 스테이지 HM0 - 모터 쪽으로 끝까지 갔다가 홈으로

                        cmds = [f'@ST0',
                                 '@MA0',
                                 '@HMV512000',
                                 '@HMA5120000',
                                 '@HMAD5120000',
                                 '@HM1']
                        [self.send_recv(num,msg= cmd) for cmd in cmds]
                        
                        # 초기화 횟수 업데이트
                        # 컨트롤러에 연결된 모든 스테이지의 카운트를 1 늘림
                        lst = self.num_of_initializations[f'ctrler_{num}']
                        self.num_of_initializations[f'ctrler_{num}']=[x + 1 if x is not None else None for x in lst]
                        
                        # 추가 구동명령 가능
                        self.ctrler_orderable[f'ctrler_{num}'] = True

                    except:
                        pass
    
    def initialize_stage(self, stage : Stage= None):
        '''특정 스테이지 위치 초기화 (원점으로 이동)'''
        with QMutexLocker(self.mutex):
            if stage != None:
                num = stage.ctrler_num
                port = stage.ctrler_port
                old_version = stage.stage_old_version # bool type
                
                # 추가 구동명령 불가
                self.ctrler_orderable[f'ctrler_{num}'] = False

                

                # 구형 스테이지 HM1 - 모터 쪽으로 끝까지 갔다가 홈으로
                # 구형 스테이지 HM0 - 아주 살짝 움직였다가 홈으로 <- 
                
                # 신형 스테이지 HM1 - 아주 살짝 움직였다가 홈으로 <
                # 신형 스테이지 HM0 - 모터 쪽으로 끝까지 갔다가 홈으로

                cmds = [f'{port}ST0',
                        f'{port}MA0',
                        f'{port}HMV512000',
                        f'{port}HMA5120000',
                        f'{port}HMAD5120000',
                        # f'{port}HM{int(not old_version)}'
                        f'{port}HM1'
                        ]
                [self.send_recv(num,msg= cmd) for cmd in cmds]

                # 초기화 횟수 업데이트
                self.num_of_initializations[f'ctrler_{num}'][port-1] += 1
                
                # 추가 구동명령 가능
                self.ctrler_orderable[f'ctrler_{num}'] = True
    
    def move_stage(self, stage: Stage= None, mode = str , speedLv = 1, coordinate = None, distance = None, direction = None):
        '''특정 스테이지 좌표, 거리, 조그 이동 '''
        # coordinate : 좌표로 이동
        # distance : 현위치에서 거리 이동
        # jog : 한쪽 방향으로 계속 이동
        
        # 현위치 최신화 한번 하고
        if stage != None:
            # _, position = self.load_step_position_of_stage(stage)
            position = stage.position
            
        
        with QMutexLocker(self.mutex):
            if stage != None:
                num = stage.ctrler_num
                port = stage.ctrler_port
                
                stage_max_position = stage.stage_max_position
                speed_limit = stage.speed_limit
                steprev = stage.steprev
                
                mode = mode.lower() # distance / coordinate / jog
                
                cmds = []
                
                # 컨트롤러 명령가능 여부 확인
                if self.ctrler_orderable[f'ctrler_{num}']:
                    # 추가 구동명령 불가로 변경
                    self.ctrler_orderable[f'ctrler_{num}'] = False
                    
                    # 조건에 따라 구동명령어 수행
                    if  speedLv <= speed_limit:
                        speed = steprev * speedLv
                        
                        if (mode == 'coordinate') & (coordinate != None):
                            target = stage.convert_position_to_step(coordinate)
                            
                            if coordinate > stage_max_position:
                                # 최대 동작 범위 밖이라서 명령 수행 불가
                                print(f'최대동작범위({stage_max_position}{stage.stage_unit}) 초과 : {coordinate}{stage.stage_unit}')
                            else:
                                cmds = [f'{port}ST0',
                                        f'{port}MA1',
                                        f'{port}A358400',
                                        f'{port}AD358400',
                                        f'{port}V{(speed)}',
                                        f'{port}D{(target)}']
                                
                        elif (mode == 'distance') & (distance != None):
                            target = stage.convert_position_to_step(distance)

                            # 만약 이동하려는 거리와 현재 위치의 합이 최대 동작 범위보다 크다면
                            if (position+distance) > stage_max_position:
                                # 최대 동작 범위 초과로 명령 수행 불가
                                print(f'최대동작범위({stage_max_position}{stage.stage_unit}) 초과 : 현위치{position}{stage.stage_unit} + 이동거리 {distance}{stage.stage_unit}')
                                
                            else:
                                cmds = [f'{port}ST0',
                                        f'{port}MA0',
                                        f'{port}A3584000',
                                        f'{port}AD358400',
                                        f'{port}V{(speed)}',
                                        f'{port}D{(target)}']
                                
                        elif mode == 'jog': # <- 버튼 클릭 구현 이후 작업
                            # 해당 스테이지의 가상 축 방향 먼저 가져와야함
                            sign_map = {True:['-', '+'], False:['+', '-']}
                            sign = sign_map[stage.stage_axis][direction]
                            
                            
                            # f'{port}V' + str(speed)
                            cmds = [f'{port}ST0',
                                    f'{port}V{(speed)}',
                                    f'{port}A358400',
                                    f'{port}AD358400',
                                    f'{port}J{sign}',
                                    f'{port}MA0']
                            
                            pass
                            
                        else:
                            # mode에 맞는 값이 아니라서 명령 수행 불가
                            pass
                        
                        [self.send_recv(num,msg= cmd) for cmd in cmds]
                        
                    else:
                        # 최대 속도 초과로 인해 명령 수행 불가
                        fastprint(f'{num} - {port} : 최대 속도({stage.speed_limit})를 초과했습니다.\n')
                    
                    # 동작 카운트 추가
                    self.num_of_movements[f'ctrler_{num}'][port-1] += 1
                    
                    # 추가 구동 명령 가능
                    self.ctrler_orderable[f'ctrler_{num}'] = True

    # 동작 명령한 스테이지를 기억해뒀다가 한번에 Go 명령어 내려야함
    # 한 축에 대해서 한번만 명령 가능
    
    
    def stop_stage(self, stage: Stage= None):
        '''특정 스테이지 긴급일시정지'''
        # 일시 정지
        # 모션은 'B' 그대로 유지됨
        # 직후 명령어 '#G' 보내면 직전 동작 다시 수행
        # 모든 명령 큐 삭제?
        with QMutexLocker(self.mutex):
            num = stage.ctrler_num
            port = stage.ctrler_port
            
            if stage != None:
                # 추가 구동 명령 불가
                self.ctrler_orderable[f'ctrler_{num}'] = True
                
                # 구동명령 확인 조건이 없는 이유는 정지가 더 우선이기 때문
                self.send_recv(num, msg=f'{port}S')
                # 업데이트 정지
            # 추가 구동 명령 가능
            self.ctrler_orderable[f'ctrler_{num}'] = True
    
    # -------------------------------------------------------------------------------------
    
    # -------------------------------------------------------------------------------------
        
    # 직선보간제어 기능
        # 한 컨트롤러 안에서만 가능
        # 작업 위치 이동 또는 렌즈 집게에서 사용 예정
    def LinearInterpolation(self):
        '''직선보간제어 구동'''
        # 각 축에 대한 스탭값을 입력으로 받음
        # 스테이지의 변환식 사용
        pass
    
    # -------------------------------------------------------------------------------------
    
    # 데이터 통신
    def send(self, num, msg:str):
        '''지정한 컨트롤러에게 메세지 보내기
            ---
            최소 2ms 간격을 두고 메세지를 보내야 합니다.
            
            Parameters:
                num (str): 명령을 수행할 컨트롤러 번호
                msg (str): 컨트롤러가 수행할 메뉴얼 상의 명령어 (ex: #3D51200G)
        '''
        
        ctrlerskt = self.ctrler_sockets[f'ctrler_{num}']
        if type(ctrlerskt) == sk.socket:
            # QTimer.singleShot(2, lambda:ctrlerskt.send(msg.encode('unicode_escape')))
            QTest.qWait(3) # 2ms
            ctrlerskt.send(msg.encode('unicode_escape'))

            # loop = QEventLoop()
            # QTimer.singleShot(2, loop.quit)
            # loop.exec_()

            # QTimer 반복문 내에서 지연시간 중첩되면서 출력 오류
    
    def send_recv(self, num, msg:str):
        '''지정한 컨트롤러와 메세지 주고 받기
            ---
            최소 3ms 간격을 두고 메세지를 주고 받아야합니다.
            
            Parameters:
                num (str): 명령을 수행할 컨트롤러 번호
                msg (str): 컨트롤러가 수행할 메뉴얼 상의 명령어 (ex: #3D51200G)
            Returns:
                data(str): 명령 수행 후 반환하는 데이터
        '''
        
        ctrlerskt = self.ctrler_sockets[f'ctrler_{num}']

        if isinstance(ctrlerskt, sk.socket):
            ctrlerskt.settimeout(1)

            try:
                if type(ctrlerskt) == sk.socket:
                    # start = time.time()
                    ctrlerskt.send(msg.encode('unicode_escape'))
                    # mid_1 = time.time()

                    # 구형 컨트롤러 메세지 반환 오류 없는 시간값
                    # QTimer 반복문 내에서 지연시간 중첩되면서 출력 오류
                    # QTest.qWait(1) # 3ms

                    data = ctrlerskt.recv(1024).decode('unicode_escape')

                    # end = time.time()

                    # fastprint(f'ctrler_{num}한테 명령어 :{msg} / 보내는 시간 {mid_1-start} 다시 받는 시간 {end - mid_1}\n')

                    return data
            except sk.timeout:
                # print(f"[Warning] 컨트롤러({num}) 응답 timeout 발생")
                return None  # 혹은 특별한 에러값/예외 발생 처리
            
            except Exception as e:
                # print(f"[Error] send_recv 예외: {e}")
                return None
            finally:
                # 필요하면 타임아웃 원상복귀 가능: ctrlerskt.settimeout(None)
                pass
        else:
            pass
    # -------------------------------------------------------------------------------------
    # 시뮬레이션 출력 생성기 - 만들면 좋음
    def Generating_Message(self, ctrler, msg):
        # @붙으면 이 컨트롤러의 전체 메세지
        # '#숫자'가 붙으면 해당 포트의 메세지
        
        # 
        
        pass
    
    # -------------------------------------------------------------------------------------
    # 사용 안함
    
    # 함수 내 try 구문이 있어야 구/신형 컨트롤러 전부 작동
    # 오류 발생시 except 구문에서 어떤 메세지 내보낼지는 아직 미정
    # 오류 발생시 pass 넣어야 스레드 재실행 가능
    
    def StageStatus(self, ctrler = int):
        '''지정된 컨트롤러에 연결된 모든 스테이지의 상태 정보'''
        try:
            statmsg = str(self.send_recv(ctrler,msg='@STATUS'))         # 반환 데이터 원문
            statidx = statmsg.find('STATUS')
            statlist = list(map(str,statmsg[statidx+7:].split(',')))    # 포트 별로 나누기
            
            for _ in range(4):
                stat = statlist[_].replace('\r','').replace('_','')     # 개행문자, 언더바 제거
                stat = ''.join(reversed(stat))
            
                statlist[_] = stat
            
            # 리스트 길이 오류 확인 필요?
            self.statuses[f'ctrler_{ctrler}'] = statlist
            
            return self.statuses
        
        except:
            pass

    def StageStep(self, ctrler = int):
        '''지정된 컨트롤러에 연결된 모든 스테이지의 스탭 정보'''
        try:
            stepsmsg = str(self.send_recv(ctrler,msg='@P'))[:-1]        # 반환 데이터 원문
            stepsidx = stepsmsg.find('POS')
            steplist = list(map(int,stepsmsg[stepsidx+3:].split(',')))  # 포트 별로 나누기
            
            self.steps[f'ctrler_{ctrler}'] = steplist
            
            # 리스트 길이 오류 확인 필요?
            return self.steps
        
        except:
            pass
            
    def StageMotion(self, ctrler = int):
        '''지정된 컨트롤러에 연결된 모든 스테이지의 모션 정보'''
        try:
            motionmsg = str(self.send_recv(ctrler,msg='@R'))[:-1]           # 반환 데이터 원문
            motionidx = motionmsg.find('R:')
            motionlist = list(map(str,motionmsg[motionidx+2:].split(',')))  # 포트 별로 나누기

            self.motions[f'ctrler_{ctrler}'] = motionlist
            
            # 리스트 길이 오류 확인 필요?
            return self.motions
        
        except:
            pass
    
    # 스테이지 정보 최신화 스레드
    def CtrlerInfoUpdateThread(self, ctrler):
        '''지정된 컨트롤러의 정보를 실시간으로 업데이트 하는 스레드'''
        
        # 컨트롤러에서 불러오기만 함
        
        # 스테이지 클래스 변수에 의한 최신화는 다른 스레드에서 진행하려다가 말았음
            # 스레드 대수 많아지면 느려지는 문제 발생
            # 이 스레드에서 각 stage 클래스로 값을 뿌림
        
        self.ctrler_orderable[f'ctrler_{ctrler}'] = False
        
        # 스탑체커
            # 스테이지 움직임이 일정 이상 없으면 명령 가능으로 바꿈
        
        stopcnt = 0
        
        # 컨트롤러 업데이트 상태 갱신
        self.ctrler_updating[f'ctrler_{ctrler}'] = True
     
        
        while not self.ctrler_orderable[f'ctrler_{ctrler}']:
            
            self.StageStatus(ctrler)
            self.StageStep(ctrler)
            self.StageMotion(ctrler)
            
            stgs = self.stages[f'ctrler_{ctrler}']
            
            if 'B' not in self.motions[f'ctrler_{ctrler}']:
                stopcnt += 1
            else:
                stopcnt = 0
            
            # 모터 클래스로 정보 보내기
            for _ in range(len(stgs)):
                if stgs[_] != None:
                    
                    step = self.steps[f'ctrler_{ctrler}'][_]
                    self.positions[f'ctrler_{ctrler}'][_] = stgs[_].convert_step_to_position(step)

                    status = self.statuses[f'ctrler_{ctrler}'][_]
                    stgs[_].statuscode_converter(statuscode=status)
                    
                    stgs[_].motion = self.motions[f'ctrler_{ctrler}'][_]
            
            # 스레드 끝내기
            if stopcnt > 50:
                self.ctrler_updating[f'ctrler_{ctrler}'] = False
                self.ctrler_orderable[f'ctrler_{ctrler}'] = True
            
            # fastprint(f'{self.motions[f'ctrler_{ctrler}']}, {stopcnt}\n')
            
        # 구동명령 불가능할 때:
            # 스탭 정보 업데이트
    
    # 스테이지 구동 가능상태 체크
    
    # -------------------------------------------------------------------------------------
        
    # 데이터 리딩 속도 개선으로 사용 안함
    def CheckStageMotionThread(self,stagename):
        '''지정된 컨트롤러에 연결된 특정 스테이지의 움직임을 판단하는 스레드'''
        
        stagectrler = int(stagename.ctrler_num)
        stageport = int(stagename.ctrler_port)
        
        # MMT 구동명령이 불가능하면
        # if not self.stage_orderable[f'ctrler_{stagectrler}']:
        #     # 멈출 때까지
        
        cnt = 0
        
        while True: # <- 조건 달아줘야함
            # 이전 스탭
            prestep = self.steps[f'ctrler_{stagectrler}'][stageport-1]
            time.sleep(0.01)
            # 0.01초 이후 스탭
            poststep = self.steps[f'ctrler_{stagectrler}'][stageport-1]
            
            if prestep==poststep:
                cnt +=1
            else:
                cnt = 0
            
            if cnt > 10:
                # 스테이지 멈춤 판단
                self.motionchecks[f'ctrler_{stagectrler}'][stageport-1] = 'S'
                # 스테이지 구동 가능상태
                stagename.stage_orderable = True
            else:
                self.motionchecks[f'ctrler_{stagectrler}'][stageport-1] = 'B'
    
    # -------------------------------------------------------------------------------------
    # 스테이지 클래스 스레드
        # 멀티 스레드 속도 저하 문제로 사용하지 않음
    def StageInfoUpdateThread(self, stagename):
        '''컨트롤러 클래스 변수로부터 자동스테이지의 정보를 실시간으로 업데이트 하는 스레드'''
        stagectrler = int(stagename.ctrler_num)
        stageport = int(stagename.ctrler_port)
        
        while True: # 조건 달아줘야함
            # 딕셔너리들로부터 정보 추출하여 부여함
            # 상태코드
            # stagename.status = self.statuses[f'ctrler_{stagectrler}'][stageport-1]
            # fastprint(self.statuses[f'ctrler_{stagectrler}'][stageport-1])
            # 스탭
            stagename.step = self.steps[f'ctrler_{stagectrler}'][stageport-1]

            # 모션
            stagename.motion = self.motionchecks[f'ctrler_{stagectrler}'][stageport-1]
    
    def StageInfoConvertThread(self, stagename):
        # stagectrler = int(stagename.ctrler_num)
        # stageport = int(stagename.ctrler_port)
        
        time.sleep(0.3)
        
        while True: # 조건 달아줘야함
            # stagename.statuscode_converter(stagename.status)
            stagename.convert_step_to_position(stagename.step)
    
    def X_StagePostion(self, ctrler= int, port= int):
        '''지정된 MMT에 연결된 스테이지 위치 정보 변환'''
        if port == 0:
            numofstage = len(self.stages[f'ctrler_{ctrler}'])
            positions = []
            for i in range(numofstage):
                step = self.steps[f'ctrler_{ctrler}'][i]
                pos = self.stages[f'ctrler_{ctrler}'][i].convert_step_to_position(step)
                positions.append(pos)
            
            self.positions[f'ctrler_{ctrler}'] = positions
            
            return positions
    
        else:
            step = self.steps[f'ctrler_{ctrler}'][port-1]
            position = self.stages[f'ctrler_{ctrler}'][port-1].convert_step_to_position(step)
            self.positions[f'ctrler_{ctrler}'][port-1] = position

            return position

    
    # -------------------------------------------------------------------------------------
    # 스테이지 구동 - stage_orderable 구현 X
    
    def StagesPowerOnOff(self, ctrler = int, port = int, pwr = str):
        '''컨트롤러 전체 또는 특정 포트의 전원을 켜거나 끔'''
        
        pwr = pwr.lower()
        
        if pwr == 'on':
            pwrorder = 0    
        elif pwr == 'off':
            pwrorder = 1
        else:
            pass
        
        # 컨트롤러의 모든 포트의 전원
        if port == 0:
            self.send(ctrler, msg = f'@ST{pwrorder}')
        
        # 1 ~ 4번 포트의 전원 컨트롤
        elif port <=4 :
            
            if self.stages[f'ctrler_{ctrler}'][port-1] != None:
            
                data = self.send_recv(ctrler, msg = f'{port}ST{pwrorder}')
            
                # fastprint(data)
            
                # fastprint(f'{port}@ST{pwrorder}')
            else:
                fastprint(f'{ctrler}번 컨트롤러의 {port}번 포트는 비어 있습니다.\n')
        else:
            fastprint(f'{ctrler}번 컨트롤러의 {port}번 포트는 없습니다.\n')

@dataclass
class StageCommand:
    '''
    컨트롤러 명령문
    
    Parameters
    ----------
        executor : Any
            명령을 실행할 대상. '컨트롤러_번호' 또는 '스테이지_이름 클래스'
        *args
            The variable arguments are used for ...
        **kwargs
            The keyword arguments are used for ...
    '''
    # number: int = 0
    # timestamp: Optional[str] = None
    
    # 누가
        # 컨트롤러_번호 또는 스테이지 클래스 : controller_1 / Stage_Name
    executor: Optional[Any] = None
    
    # 무엇을
    command: Optional[str] = None
        # connect / disconnect / change / refresh / (power)
        # move + start / initialize / home / jog
        # stop
    
    # 어디에(connect/change 커맨드에서만 가능)
        # 컨트롤러_번호_포트 또는 'PC' : controller_1_4 / PC
        # 변경하려는 대상(네트워크 설정값) : ip, subnetmask, gateway, port
    target: Optional[str] = None
    
    # 어떻게
        # 스테이지 명령
    movement: Dict[str, Any] = field(default_factory=lambda: {'mode': None,                 # coordinate / distance / jog
                                                              'speed': None,                # mm/s
                                                              'value': None,                # 좌표 또는 상대거리/각
                                                              'direction' : None,           # 조그 방향
                                                              'operation_delay_ms' : 0})    # 구동 종료 후 다음 구동 전까지 기다림
    
    connection : Dict[str, Any] = field(default_factory=lambda: {'simulated': False,            # 가상 장비 여부
                                                                 'old': False,                  # 구형 버전 여부
                                                                 'changed_ip' : None,           # 네트워크 - 아이피 변경
                                                                 'changed_subnetmask' : None,   # 네트워크 - 서브넷마스크 변경
                                                                 'changed_gateway' : None,      # 네트워크 - 게이트웨이 변경
                                                                 'changed_port': None})         # 네트워크 - 포트 변경
    
        # 전원 on = True / off = False
    power: Optional[bool] = None

    # 왜
        # 명령 설명
    message: Optional[str] = None
    
    def __post_init__(self):
        # movement 병합
        default_movement = {'mode': None,
                            'speed': None,
                            'value': None,
                            'direction': bool,
                            'operation_delay_ms': 0}
        if self.movement is not None:
            merged = default_movement.copy()
            merged.update(self.movement)
            self.movement = merged

        # connection 병합
        default_connection = {'simulated': False,
                              'old': False,
                              'changed_ip' : None,
                              'changed_subnetmask' : None,
                              'changed_gateway' : None,
                              'changed_port': None}
        
        if self.connection is not None:
            merged = default_connection.copy()
            merged.update(self.connection)
            self.connection = merged
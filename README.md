### 날짜 :  2024-02-09 / 02:20

### 주제 : #UWB #triangulation
---
### 주요 기능

- **anchor와 tag 사이의 거리 publish**
- **삼각측량법(triangulation) 이용해서 로봇 기준 tag의 좌표 추정**
	triangulation.get_pose()
- **로봇 기준 tag 위치를 map 기준으로 좌표 표현**
	 tag_position.get_tf()
		 main branch -> tag 위치를 구함.
		 test branch -> tag 위치의 1m 앞을 구함.
- **move_base_simple/goal publish**
	 tag_position.publish_current_goal()
	 => self.callback_interval 만큼의 주기 보장(더 느리게 pub 할 수도 있음)

### 기타 기능

- filter
	kalman, moving avg 두 가지에 대해서 구현 (filter.py 확인)
	24.02.09 기준 triangulation 과정에서 구한 get_angle 결과에만 movavgfilter 적용 중
- tag_position publish
	사실 tag 위치를 바로 goal로 pub 하기 때문에 굳이 tag 위치를 pub 할 필요 없으나,  base_link 기준 tag 위치와 map 기준 tag 위치를 rviz 상에 보고자 tag 위치 publish 내용 포함
	 => 추후 삭제 필요
- tag_position.stop()
	로봇 위치(amcl_poose)와 tag 위치가 1m 이내에 들어올 경우 로봇의 위치를 pub하고 flag를 STOP으로 바꿔 계속해서 publish 되지 않도록 함.

---

### 개선 필요 사항

- [ ] tag 위치 이동 시, 값이 튀는 현상 존재 -> UWB 데이터가 동시에 들어오지 않아서로 추측 중. 동시에 혹은 최대한 가까운 시간끼리 데이터 맞출 필요 있음.
- [ ] stop() 함수 제대로 작동하는 지 검증
- [ ] 좁은 공간에서 앞으로 제대로 움직이지 않아서 넓은 공간에서 테스트 필요

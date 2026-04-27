import threading
from typing import Dict, Any
from ultralytics.trackers.byte_tracker import BYTETracker
import logging

logger = logging.getLogger(__name__)

class TrackerManager:
    def __init__(self, tracker_args: Dict[str, Any] = None):
        self.trackers: Dict[str, BYTETracker] = {}
        self.lock = threading.Lock()

        # 初始化跟踪器参数
        if tracker_args:
            self.tracker_args = type('Args', (), tracker_args)()
        else:
            self.tracker_args = type('Args', (), {
            'track_high_thresh': 0.5,
            'track_low_thresh': 0.1,
            'new_track_thresh': 0.3,
            'track_buffer': 100,
            'match_thresh': 0.6,
            'fuse_score': True
        })()
    
    def get_tracker(self, dataId: str) -> BYTETracker:
        """
        为指定的dataId获取或创建跟踪器
        """
        with self.lock:
            if dataId not in self.trackers:
                self.trackers[dataId] = BYTETracker(self.tracker_args)
                if logger:
                    logger.info(f"Created new tracker for stream {dataId}")
            return self.trackers[dataId]
    
    def reset_tracker(self, dataId: str):
        """
        重置指定dataId的跟踪器
        """
        with self.lock:
            if dataId in self.trackers:
                self.trackers[dataId].reset()
                if logger:
                    logger.info(f"Reset tracker for stream {dataId}")
    
    def remove_tracker(self, dataId: str):
        """
        移除指定dataId的跟踪器
        """
        with self.lock:
            if dataId in self.trackers:
                del self.trackers[dataId]
                if logger:
                    logger.info(f"Removed tracker for stream {dataId}")
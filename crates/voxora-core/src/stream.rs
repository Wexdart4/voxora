//! Bounded frame queue and streaming memory management.

use crate::Frame;
use std::collections::VecDeque;

/// Thread-safe bounded queue for streaming video frame processing.
#[derive(Debug, Clone)]
pub struct BoundedFrameQueue {
    /// Maximum capacity of frame queue
    pub max_capacity: usize,
    /// Internal queue buffer
    queue: VecDeque<Frame>,
}

impl BoundedFrameQueue {
    /// Creates a bounded frame queue with specified capacity.
    pub fn new(max_capacity: usize) -> Self {
        Self { max_capacity: max_capacity.max(1), queue: VecDeque::with_capacity(max_capacity) }
    }

    /// Pushes a frame into the queue. Drops the oldest frame if capacity is exceeded.
    pub fn push(&mut self, frame: Frame) -> Option<Frame> {
        let dropped =
            if self.queue.len() >= self.max_capacity { self.queue.pop_front() } else { None };

        self.queue.push_back(frame);
        dropped
    }

    /// Pops the next frame from queue.
    pub fn pop(&mut self) -> Option<Frame> {
        self.queue.pop_front()
    }

    /// Current number of frames in queue.
    pub fn len(&self) -> usize {
        self.queue.len()
    }

    /// Whether queue is empty.
    pub fn is_empty(&self) -> bool {
        self.queue.is_empty()
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::PixelFormat;

    #[test]
    fn test_bounded_frame_queue() {
        let mut queue = BoundedFrameQueue::new(2);

        let f1 = Frame::new(10, 10, PixelFormat::Grayscale, vec![0; 100]).unwrap();
        let f2 = Frame::new(10, 10, PixelFormat::Grayscale, vec![1; 100]).unwrap();
        let f3 = Frame::new(10, 10, PixelFormat::Grayscale, vec![2; 100]).unwrap();

        assert!(queue.push(f1).is_none());
        assert!(queue.push(f2).is_none());

        let dropped = queue.push(f3).unwrap();
        assert_eq!(dropped.data[0], 0);
        assert_eq!(queue.len(), 2);
    }
}

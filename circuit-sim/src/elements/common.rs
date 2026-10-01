//! State shared by every element: geometry, node/voltage arrays, and the current.

use crate::element::Point;

/// The parts of `CircuitElm` that the engine needs, minus drawing and editing.
#[derive(Clone, Debug)]
pub struct Base {
    pub x: i32,
    pub y: i32,
    pub x2: i32,
    pub y2: i32,
    pub flags: i32,
    /// 1-based node numbers, `post_count() + internal_node_count()` entries.
    pub nodes: Vec<usize>,
    /// Voltage at each of those nodes.
    pub volts: Vec<f64>,
    pub current: f64,
    /// Index of this element's (single) voltage source row, set by the engine.
    pub volt_source: usize,
}

impl Base {
    pub fn new(x: i32, y: i32, x2: i32, y2: i32, flags: i32, node_slots: usize) -> Self {
        Self {
            x,
            y,
            x2,
            y2,
            flags,
            nodes: vec![0; node_slots],
            volts: vec![0.0; node_slots],
            current: 0.0,
            volt_source: 0,
        }
    }

    /// An element spanning two [`Point`]s, which is how circuits are actually written down.
    ///
    /// The post coordinates are load-bearing rather than cosmetic: the engine identifies a
    /// node purely by the position of the post that lands on it, which is how two elements
    /// drawn sharing a terminal become the same node without any explicit wiring.
    pub fn from_points(p1: Point, p2: Point, flags: i32, node_slots: usize) -> Self {
        Self::new(p1.x, p1.y, p2.x, p2.y, flags, node_slots)
    }

    pub fn point1(&self) -> Point {
        Point::new(self.x, self.y)
    }

    pub fn point2(&self) -> Point {
        Point::new(self.x2, self.y2)
    }

    pub fn voltage_diff(&self) -> f64 {
        self.volts[0] - self.volts[1]
    }

    pub fn power(&self) -> f64 {
        self.voltage_diff() * self.current
    }

    /// Resize the node arrays when a subclass reports a different slot count.
    pub fn ensure_slots(&mut self, n: usize) {
        if self.nodes.len() != n {
            self.nodes.resize(n, 0);
            self.volts.resize(n, 0.0);
        }
    }

    /// Port of `CircuitElm.reset`: zero the node voltages and the current.
    pub fn reset(&mut self) {
        for v in self.volts.iter_mut() {
            *v = 0.0;
        }
        self.current = 0.0;
    }
}

/// Implements the boilerplate half of [`Element`] in terms of a `base` field and a
/// `calculate_current` method on the concrete type.
///
/// Every element has to answer the same eight questions — how many posts, where they are,
/// which node each maps to, what voltage arrived, what current flows — and the answers only
/// ever touch `Base`. This macro supplies them so each element file is left with just its own
/// physics and kind predicates.
///
/// The element-specific trait methods go in the `extra = { ... }` block. The braces are
/// matched literally by the pattern and then destructured into `item`s, which is what lets
/// them be spliced into the generated `impl` as individual associated items. Capturing the
/// whole thing as a single `tt` would not work: re-emitting it would restore the braces and
/// the parser would see a block expression rather than a list of trait items.
///
/// A type using this needs a `base: Base` field, a `from_base(Base) -> Self`, and a
/// `calculate_current(&mut self)`.
macro_rules! impl_element {
    (
        $ty:ty,
        posts = $posts:expr,
        extra = { $($extra:item)* }
    ) => {
        impl $ty {
            pub fn base(&self) -> &$crate::elements::Base {
                &self.base
            }

            pub fn base_mut(&mut self) -> &mut $crate::elements::Base {
                &mut self.base
            }
        }

        impl $crate::element::Element for $ty {
            fn as_any(&self) -> &dyn std::any::Any {
                self
            }

            fn as_any_mut(&mut self) -> &mut dyn std::any::Any {
                self
            }

            fn post_count(&self) -> usize {
                $posts
            }

            fn post(&self, n: usize) -> $crate::element::Point {
                debug_assert!(n < $posts);
                if n == 0 {
                    self.base.point1()
                } else if n == 1 {
                    self.base.point2()
                } else {
                    self.extra_post(n)
                }
            }

            fn set_node(&mut self, p: usize, n: usize) {
                self.base.nodes[p] = n;
            }

            fn node(&self, n: usize) -> usize {
                self.base.nodes[n]
            }

            fn set_voltage_source(&mut self, _n: usize, v: usize) {
                self.base.volt_source = v;
            }

            fn set_node_voltage(&mut self, n: usize, v: f64) {
                self.base.volts[n] = v;
                self.calculate_current();
            }

            fn set_current(&mut self, _n: usize, c: f64) {
                let c = self.adjust_solved_current(c);
                self.base.current = c;
            }

            fn current(&self) -> f64 {
                self.base.current
            }

            fn reset(&mut self) {
                self.base.reset();
                self.reset_extra();
            }

            $($extra)*
        }
    };
}

pub(crate) use impl_element;

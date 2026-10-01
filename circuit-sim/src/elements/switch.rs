//! SPST switch. Port of `SwitchElm.java`.
//!
//! A closed switch is a 0 V voltage source, which is the cheapest way to get "these two nodes
//! are the same node" out of MNA: it adds a row that forces the two node voltages together
//! and gives the current a place to be reported. An open switch stamps nothing at all, so it
//! contributes no row and no connection — which is why [`Switch::voltage_source_count`] and
//! [`Switch::get_connection`] both key off the position.

use crate::element::SimContext;
use crate::elements::common::{impl_element, Base};
use crate::stamper::Stamper;

/// A single-pole, single-throw switch.
///
/// `position == 0` is closed and `position == 1` is open, matching the Java field. The
/// engine never changes the position on its own; [`Switch::toggle`] is what a UI or a test
/// calls, and it leaves `analyzeFlag` set so the next step re-runs analysis with the new
/// topology.
#[derive(Clone, Debug)]
pub struct Switch {
    base: Base,
    position: u8,
    momentary: bool,
}

impl Switch {
    pub fn from_base(base: Base, position: u8, momentary: bool) -> Self {
        Self {
            base,
            position,
            momentary,
        }
    }

    /// A closed switch.
    pub fn closed(p1: crate::element::Point, p2: crate::element::Point) -> Self {
        Self::from_base(Base::from_points(p1, p2, 0, 2), 0, false)
    }

    /// An open switch.
    pub fn open(p1: crate::element::Point, p2: crate::element::Point) -> Self {
        Self::from_base(Base::from_points(p1, p2, 0, 2), 1, false)
    }

    /// Flips the position. Port of `SwitchElm.toggle`.
    pub fn toggle(&mut self) {
        self.position = (self.position + 1) % 2;
    }

    /// 0 when closed, 1 when open.
    pub fn position(&self) -> u8 {
        self.position
    }

    pub fn is_closed(&self) -> bool {
        self.position == 0
    }

    pub fn is_momentary(&self) -> bool {
        self.momentary
    }

    pub fn set_momentary(&mut self, momentary: bool) {
        self.momentary = momentary;
    }

    /// Port of `SwitchElm.calculateCurrent`: an open switch carries nothing.
    ///
    /// A closed switch's current comes from the solved voltage-source row, so there is
    /// nothing to compute here.
    fn calculate_current(&mut self) {
        if self.position == 1 {
            self.base.current = 0.0;
        }
    }
}

impl_element!(
    Switch,
    posts = 2,
    extra = {
        /// An open switch contributes no row at all, so it must not claim a voltage-source slot.
        /// Port of `SwitchElm.getVoltageSourceCount`.
        fn voltage_source_count(&self) -> usize {
            usize::from(self.position == 0)
        }

        /// An open switch does not connect its posts, which is what stops the unconnected-node
        /// search from propagating closure through it. Port of `SwitchElm.getConnection`.
        fn get_connection(&self, _n1: usize, _n2: usize) -> bool {
            self.position == 0
        }

        /// A closed switch shorts its posts, so it has to be traversable by the same searches
        /// that walk a wire — but it is not a wire, so it keeps its own matrix row and its own
        /// current. Port of `SwitchElm.isWire`, which the Java original consults from the
        /// voltage-loop check and `FindPathInfo.findPath`.
        fn is_wire_like(&self) -> bool {
            self.position == 0
        }

        /// Port of `SwitchElm.stamp`: a closed switch is a 0 V source between its posts.
        fn stamp(&mut self, stamper: &mut Stamper, _ctx: &SimContext) {
            if self.position == 0 {
                let (n0, n1) = (self.base.nodes[0], self.base.nodes[1]);
                stamper.stamp_voltage_source(n0, n1, self.base.volt_source, 0.0);
            }
        }
    }
);

#[cfg(test)]
mod tests {
    use super::*;
    use crate::element::Element;

    #[test]
    fn open_switch_claims_no_voltage_source() {
        let mut s = Switch::open(
            crate::element::Point::new(0, 0),
            crate::element::Point::new(0, 40),
        );
        assert_eq!(Element::voltage_source_count(&s), 0);
        assert!(!Element::get_connection(&s, 1, 2));

        s.toggle();
        assert_eq!(Element::voltage_source_count(&s), 1);
        assert!(Element::get_connection(&s, 1, 2));
        assert!(Element::is_wire_like(&s));
    }
}

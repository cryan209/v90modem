"""Bundle discovery must survive renamed interfaces and a departed leader."""
import json
from pathlib import Path
import unittest
from unittest.mock import patch

import collect_ppp_members as collector


class BundleDiscoveryTest(unittest.TestCase):
    def collect(self, leader=True, previous_index=None):
        interface = dict(ifname='e79p4', ifindex=702, link_type='ppp',
                         stats64=dict(tx=dict(bytes=120), rx=dict(bytes=60)))
        files = {
            '/etc/ppp/peers/eicon-7920-15': 'ifname e79p5\nmultilink\n',
            '/run/ppp-stats/e79p4.json': json.dumps(
                dict(ifindex=702, modem_device='ttyds14')),
            '/run/ppp-modem/ttyds15.report':
                'CONNECT 9600\nCONNECT V90/50666:TX/26400:RX\n',
        }
        devices = {'/dev/ttyds15': 15}
        if leader:
            files['/etc/ppp/peers/eicon-7920-14'] = 'ifname e79p4\nmultilink\n'
            devices['/dev/ttyds14'] = 14
        if previous_index is not None:
            files['/run/ppp-stats/members.json'] = json.dumps(dict(bundles={
                'e79p4': dict(ifindex=previous_index, members=[dict(ppp_unit=0)])}))

        def read(path, *args, **kwargs):
            if str(path) not in files:
                raise FileNotFoundError(str(path))
            return files[str(path)]

        def unit(fd, request, value, mutate):
            value[0] = 0  # Both modems join the same kernel PPP unit.

        with patch.object(Path, 'read_text', read), \
                patch.object(collector.subprocess, 'check_output', return_value=json.dumps([interface])), \
                patch.object(collector.os, 'open', side_effect=lambda name, flags: devices[name]), \
                patch.object(collector.os, 'close'), \
                patch.object(collector.fcntl, 'ioctl', side_effect=unit):
            return collector.collect()['bundles']

    def test_joined_member_does_not_need_its_configured_interface(self):
        bundle = self.collect()['e79p4']
        self.assertEqual([row['device'] for row in bundle['members']], ['ttyds14', 'ttyds15'])
        self.assertEqual(bundle['members'][1]['modem_download_bps'], 50666)
        self.assertEqual(bundle['download_bytes'], 120)

    def test_existing_bundle_survives_first_modem_departure(self):
        bundle = self.collect(leader=False, previous_index=702)['e79p4']
        self.assertEqual([row['device'] for row in bundle['members']], ['ttyds15'])

    def test_reused_interface_name_does_not_reuse_old_bundle_mapping(self):
        self.assertEqual(self.collect(leader=False, previous_index=701), {})


if __name__ == '__main__':
    unittest.main()

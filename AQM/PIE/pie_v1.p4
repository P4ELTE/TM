/*
Copyright 2018 ELTE Eötvös Loránd University
Licensed under the Apache License, Version 2.0 (the "License");
you may not use this file except in compliance with the License.
You may obtain a copy of the License at
    http://www.apache.org/licenses/LICENSE-2.0
Unless required by applicable law or agreed to in writing, software
distributed under the License is distributed on an "AS IS" BASIS,
WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
See the License for the specific language governing permissions and
limitations under the License.

Authors: 
	* Sandor Laki, lakis@elte.hu

*/

#include <core.p4>
#include <v1model.p4>

/*
 * Proportional Integral Controller Enhanced (PIE) AQM is described in:
 *	Paper: https://ieeexplore.ieee.org/abstract/document/6602305
 *	RFC 8033: https://tools.ietf.org/html/rfc8033 
 *	Linux Kernel implementation: https://elixir.bootlin.com/linux/v4.13-rc7/source/net/sched/sch_pie.c
 *
 * Version 1: Fast path only implementation
 *
 *
*/

/* Typedefs */
typedef bit<9> queueId_t;
typedef bit<32> queueDepth_t;
typedef bit<32> avgDqRate_t;
typedef bit<32> uint_t;
typedef bit<10> log_uint_t;


/* Number of queues */
const bit<32> N_QUEUES = 256;

/* PIE parameters */
const bit<48> cTimeUpdate = 30000000; // Update time in ns (30 ms); timer frequency
const uint_t cTarget = 10000000; // Target delay in ns (10 ms)
const uint_t cMaxProb =0xffffffff;
const bit<8> cPieScale = 8;
const queueDepth_t cLimit = 1000; // 1000 packets ; it could also be part of the buffer extern
const bool cECN = false; // ECN support on-1/off-0

/* Parameters of the PI controller */
const uint_t cAlpha = 2; 
const uint_t cBeta = 20;


extern buffer {
    buffer();
    queueDepth_t get_qsize(in queueId_t index); /* Queue size in bytes*/
    queueDepth_t get_qlen(in queueId_t index);  /* Number of packets in the queue */
    avgDqRate_t get_drainrate(in queueId_t index); /* Average drain rate of the queue */
}

typedef bit<48>  EthernetAddress;

header Ethernet_h {
    EthernetAddress dstAddr;
    EthernetAddress srcAddr;
    bit<16>         etherType;
}

struct Parsed_packet {
    Ethernet_h    ethernet;
}

struct metadata_t {
    bit<1> unicast;
    queueId_t qid;
}

parser parserI(packet_in pkt,
               out Parsed_packet hdr,
               inout metadata_t meta,
               inout standard_metadata_t stdmeta) {
    state start {
        pkt.extract(hdr.ethernet);
        transition accept;
    }
}

control DeparserI(packet_out packet,
                  in Parsed_packet hdr) {
    apply { packet.emit(hdr.ethernet); }
}

/*
 * This code is comming from https://github.com/sibanez12/AQM-P4-examples/blob/master/GSP/adaptive-gsp-prototype.p4
 *
 * Here we will use lookup tables to approximate integer division.
 * We will use the following useful fact:
 *     A/B = exp(log(A) - log(B))
 * We will use ternary tables to approximate log() and exact match
 * tables to implement exp()
 * See this paper for more details:
 *     https://homes.cs.washington.edu/~arvind/papers/flexswitch.pdf
 */
control divide_pipe(in uint_t numerator,
                    in uint_t denominator,
                    out uint_t result) {

    log_uint_t log_num;
    log_uint_t log_denom;
    log_uint_t log_result;

    action set_log_num(log_uint_t presult) {
        log_num = presult;
    }

    table log_numerator {
        key = { numerator: ternary; }
        actions = { set_log_num; }
        size = 1024;
        default_action = set_log_num(0);
    }

    action set_log_denom(log_uint_t presult) {
        log_denom = presult;
    }

    table log_denominator {
        key = { denominator: ternary; }
        actions = { set_log_denom; }
        size = 1024;
        default_action = set_log_denom(0);
    }

    action set_result(uint_t presult) {
        result = presult;
    }

    table exp {
        key = { log_result: exact; }
        actions = { set_result; }
        size = 2048;
        default_action = set_result(0);
    }

    apply {
        // numerator / denominator = exp(log(numerator) - log(denominator))
        if (numerator == 0 || denominator == 0 || denominator > numerator) {
            result = 0;
        } else {
            log_numerator.apply();
            log_denominator.apply();
            log_result = log_num - log_denom;
            exp.apply();
        }
    }
}





control cIngress(inout Parsed_packet hdr,
                 inout metadata_t meta,
                 inout standard_metadata_t stdmeta)
{
    // externs
    buffer() buffer_inst;

    // metadata
    queueDepth_t qlen_bytes;
    queueDepth_t qlen_pkts;

    bit<48> time_next;
    bit<48> now;
    uint_t qdelay_old;
    uint_t qdelay;
    uint_t prob;
    uint_t prob_old;
    int<32> delta;
    avgDqRate_t avgrate;

    // states
    register<uint_t>(N_QUEUES) qdelay_reg;
    register<bit<48>>(N_QUEUES) time_next_reg;
    register<uint_t>(N_QUEUES) prob_reg;

    // control for division
    divide_pipe() divide;

    apply {
        /*
         * Ingress processing steps indepdent of the AQM go here.
         */

        qlen_pkts = buffer_inst.get_qlen(meta.qid);

        if (qlen_pkts>=cLimit) {
                mark_to_drop();
	}
        else {
                qlen_bytes = buffer_inst.get_qsize(meta.qid);
                prob_reg.read(prob, (bit<32>) meta.qid);
                time_next_reg.read(time_next, (bit<32>) meta.qid);
                now = stdmeta.ingress_global_timestamp;
                if ( now >= time_next ) {
                   /* update probability */
                   avgrate = buffer_inst.get_drainrate(meta.qid);
                   qdelay_reg.read(qdelay_old, (bit<32>) meta.qid);
                   prob_reg.read(prob, (bit<32>) meta.qid);
                   delta = 0;
                   if (avgrate>0) {
                      /*
                       * compute: qdelay = (qlen_bytes << cPieScale) / avgrate;
                       */
                       divide.apply(qlen_bytes << cPieScale, avgrate, qdelay);
                   } else {
                      qdelay = 0;
                   }

                   /* TODO: alpha/beta scaling is not implemented */

                   delta = delta + (int<32>) ( cAlpha * (qdelay - cTarget) );
                   delta = delta + (int<32>) ( cBeta * (qdelay - qdelay_old) );

                   /* TODO: increase probability in steps of no more than 2% */
                   if ((delta > (int<32>) (cMaxProb / (100 / 2))) && (prob >= cMaxProb / 10)) {
                        delta = (int<32>) (cMaxProb / (100 / 2)); /* set to 2% */
                   }

                   /* TODO: non-linear dropping */

                   prob_old = prob;

                   prob = prob + (bit<32>)delta; /* TODO: check arithmetic */
                   if (delta>0) {
                      if (prob < prob_old) {
                          prob = cMaxProb;
                      }
                   } else {
                      if (prob > prob_old) {
                          prob = 0;
                      }
                   }

                   /* update states */
                   prob_reg.write((bit<32>) meta.qid, prob);
                   qdelay_reg.write((bit<32>) meta.qid, qdelay);
                   time_next_reg.write((bit<32>) meta.qid,time_next + cTimeUpdate);
                }
                bit<32> rand_val;
                random<bit<32>>(rand_val, 0, cMaxProb);
                if (rand_val < prob) {
                     if (cECN && (prob <= cMaxProb/10)) {
                         /* TODO: ECN mark instead of dropping */
                         mark_to_drop();
                     }
                     else {
                         mark_to_drop();
                     }
                }
                /* enqueue packet */
	}
    }

}

control cEgress(inout Parsed_packet hdr,
                inout metadata_t meta,
                inout standard_metadata_t stdmeta) {
    apply { }
}

control vc(inout Parsed_packet hdr,
           inout metadata_t meta) {
    apply { }
}

control uc(inout Parsed_packet hdr,
           inout metadata_t meta) {
    apply { }
}


V1Switch(parserI(),
    vc(),
    cIngress(),
    cEgress(),
    uc(),
    DeparserI()) main;

